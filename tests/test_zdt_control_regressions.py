from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest
import numpy as np
from waldo_commander.components import control
from waldo_commander import main as commander_main
from waldo_commander.components.playback import PlaybackController
from waldo_commander.operator_messages import operator_error
from waldo_commander.services.urdf_scene.urdf_scene import (
    _joint_limit_label_text,
    _visual_mesh_scale,
)
from waldo_commander.services.urdf_scene.envelope_renderer import (
    _nearest_hull_boundary,
)
from waldo_commander import robot_limits
from waldo_commander.robot_limits import effective_joint_limits_deg


def test_incremental_joint_moves_allow_slow_hardware_settling() -> None:
    assert control.ControlPanel.INCREMENTAL_MOVE_TIMEOUT_S == 120.0
    assert control.ControlPanel.EXACT_MOVE_TIMEOUT_S == 120.0


def test_pose_alignment_only_enters_virtual_scene(monkeypatch) -> None:
    calls: list[str] = []
    scene = SimpleNamespace(start_pose_alignment=lambda: calls.append("alignment"))
    monkeypatch.setattr(control.ui_state, "urdf_scene", scene)
    monkeypatch.setattr(control, "is_any_program_running", lambda: False)
    monkeypatch.setattr(
        control.ui,
        "notify",
        lambda *args, **kwargs: calls.append("notify"),
    )

    panel = object.__new__(control.ControlPanel)
    panel._start_pose_alignment()

    assert calls == ["alignment", "notify"]


def test_urdf_visual_mesh_scale_preserves_millimeter_gripper_scale() -> None:
    visual = SimpleNamespace(
        geometry=SimpleNamespace(geometry=SimpleNamespace(scale=np.array([0.001] * 3)))
    )

    assert _visual_mesh_scale(visual, 1.0) == (0.001, 0.001, 0.001)
    assert _visual_mesh_scale(
        SimpleNamespace(geometry=SimpleNamespace(geometry=SimpleNamespace(scale=None))),
        1.0,
    ) == (1.0, 1.0, 1.0)


def test_zdt_speed_rating_spans_the_encodable_range() -> None:
    assert control._normalized_speed(10, "parol6_zdt_backend") == pytest.approx(0.1)
    assert control._normalized_speed(50, "parol6_zdt_backend") == pytest.approx(0.5)
    assert control._normalized_speed(100, "parol6_zdt_backend") == pytest.approx(1.0)


def test_cartesian_axis_lookup_uses_selected_reference_frames(monkeypatch) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._translation_frame = "TRF"
    panel._rotation_frame = "WRF"
    panel._cart_axis_lookup = None
    monkeypatch.setattr(
        control.ui_state,
        "robot",
        SimpleNamespace(cartesian_frames=["WRF", "TRF"]),
    )

    lookup = panel._get_cart_axis_lookup()

    assert lookup["X+"] == ("X", 1.0, "TRF")
    assert lookup["RZ-"] == ("RZ", -1.0, "WRF")


@pytest.mark.asyncio
async def test_multiturn_home_restore_writes_fixed_request_path(
    monkeypatch, tmp_path
) -> None:
    request_path = tmp_path / "home-restore.request"
    monkeypatch.setattr(control, "_HOME_RESTORE_REQUEST_PATH", request_path)

    await control._request_multiturn_home_restore()

    assert int(request_path.read_text(encoding="utf-8")) > 0


@pytest.mark.asyncio
async def test_calibration_home_uses_confirmed_hardware_pose(monkeypatch) -> None:
    assert control._ZDT_CALIBRATION_HOME_JOINTS_DEG == pytest.approx(
        (
            0.0102996826171875,
            -95.31943873355263,
            106.01783752441406,
            -0.087890625,
            -41.873931884765625,
            90.01020159040179,
        )
    )
    calls: list[tuple[list[float], dict[str, object]]] = []

    class Client:
        async def move_j(self, angles, **kwargs):
            calls.append((angles, kwargs))
            return 1

    panel = object.__new__(control.ControlPanel)
    panel.client = Client()
    panel._movement_allowed = lambda: True

    async def run(_label, operation):
        await operation()

    panel._run_incremental_move = run
    monkeypatch.setattr(control, "_norm_speed", lambda: 0.5)
    monkeypatch.setattr(control, "_norm_accel", lambda: 1.0)
    monkeypatch.setattr(control.ui, "notify", lambda *args, **kwargs: None)

    await panel._execute_calibration_home_move()

    assert calls == [
        (
            list(control._ZDT_CALIBRATION_HOME_JOINTS_DEG),
            {"speed": 0.5, "accel": 1.0, "wait": True, "timeout": 120.0},
        )
    ]


@pytest.mark.asyncio
async def test_pre_grasp_uses_saved_current_hardware_pose(monkeypatch) -> None:
    assert control._ZDT_PRE_GRASP_JOINTS_DEG == pytest.approx(
        (
            0.1338958740234375,
            -73.07270250822368,
            165.7317352294922,
            0.08514404296875,
            -75.18722534179688,
            89.96939522879464,
        )
    )
    calls: list[tuple[list[float], dict[str, object]]] = []

    class Client:
        async def move_j(self, angles, **kwargs):
            calls.append((angles, kwargs))
            return 1

    panel = object.__new__(control.ControlPanel)
    panel.client = Client()
    panel._movement_allowed = lambda: True

    async def run(_label, operation):
        await operation()

    panel._run_incremental_move = run
    monkeypatch.setattr(control, "_norm_speed", lambda: 0.5)
    monkeypatch.setattr(control, "_norm_accel", lambda: 1.0)
    monkeypatch.setattr(control.ui, "notify", lambda *args, **kwargs: None)

    await panel._execute_pre_grasp_move()

    assert calls == [
        (
            list(control._ZDT_PRE_GRASP_JOINTS_DEG),
            {"speed": 0.5, "accel": 1.0, "wait": True, "timeout": 120.0},
        )
    ]


def test_pre_grasp_starts_with_one_button_click(monkeypatch) -> None:
    panel = object.__new__(control.ControlPanel)
    started: list[str] = []

    async def move() -> None:
        started.append("move")

    def schedule(coroutine) -> None:
        started.append("scheduled")
        coroutine.close()

    panel._execute_pre_grasp_move = move
    monkeypatch.setattr(control, "_safe_task", schedule)
    monkeypatch.setattr(control.waldoctl.commander.status, "editing_mode", False)

    panel.send_pre_grasp()

    assert started == ["scheduled"]


@pytest.mark.asyncio
async def test_zdt_hardware_home_routes_to_calibration_home(monkeypatch) -> None:
    panel = object.__new__(control.ControlPanel)
    calls: list[str] = []
    panel.confirm_calibration_home_move = lambda: calls.append("calibration-home")

    monkeypatch.setattr(
        type(control.ui_state.active_robot),
        "backend_package",
        property(lambda _self: "parol6_zdt_backend"),
    )
    monkeypatch.setattr(control.waldoctl.commander.status, "editing_mode", False)
    monkeypatch.setattr(control.waldoctl.commander.status, "simulator_active", False)

    await panel.send_home()

    assert calls == ["calibration-home"]


def test_joint_direction_remains_available_for_partial_final_step() -> None:
    assert control._joint_has_remaining_travel(-98.95, -101.25, "neg")
    assert control._joint_has_remaining_travel(91.32, 81.21, "neg")
    assert not control._joint_has_remaining_travel(-101.25, -101.25, "neg")
    assert not control._joint_has_remaining_travel(228.87, 228.87, "pos")


def test_joint_commands_keep_braking_margin_from_soft_limits() -> None:
    assert control._joint_command_bounds(-10.0, 10.0) == (-9.5, 9.5)


def test_zdt_workspace_uses_deployed_soft_limits() -> None:
    robot = SimpleNamespace(
        backend_package="parol6_zdt_backend",
        joints=SimpleNamespace(
            limits=SimpleNamespace(
                position=SimpleNamespace(deg=np.asarray([[-999.0, 999.0]] * 6))
            )
        ),
    )
    limits = effective_joint_limits_deg(robot)
    assert limits[1].tolist() == pytest.approx([-137.0, -37.098302649122786])
    assert limits[4].tolist() == pytest.approx([-90.0, 3.163844])


def test_zdt_workspace_reads_installed_joint_limit_file(
    tmp_path, monkeypatch
) -> None:
    limits_path = tmp_path / "joint-limits.json"
    limits_path.write_text(
        '{"joint_limits_deg":[[-117,137],[-137,-37],[96,243],[-23,33],[-90,3],[7,192]],'
        '"schema":"parol6-zdt/joint-limits/v1"}',
        encoding="utf-8",
    )
    monkeypatch.setattr(robot_limits, "ZDT_JOINT_LIMITS_PATH", limits_path)
    robot = SimpleNamespace(backend_package="parol6_zdt_backend")

    assert effective_joint_limits_deg(robot)[4].tolist() == [-90.0, 3.0]


def test_nearest_workspace_boundary_distance() -> None:
    equations = np.asarray(
        [
            [1.0, 0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0, -1.0],
            [0.0, -1.0, 0.0, -1.0],
            [0.0, 0.0, 1.0, -1.0],
            [0.0, 0.0, -1.0, -1.0],
        ]
    )
    distance, boundary, inside = _nearest_hull_boundary(
        np.asarray([0.25, 0.0, 0.0]), equations
    )
    assert inside is True
    assert distance == pytest.approx(0.75)
    assert boundary.tolist() == pytest.approx([1.0, 0.0, 0.0])


def test_joint_limit_scene_label_reports_both_remaining_directions() -> None:
    text, values = _joint_limit_label_text(5, -52.9, -57.539281, 3.163844)
    assert values == (-52.9, 4.6, 56.1)
    assert text == "J5 -52.9°  −余4.6° / +余56.1°"


@pytest.mark.parametrize(
    "detail",
    [
        "J2 soft limit was crossed",
        "OFFICIAL_TRAJECTORY_INVALID: Joint 2 target is out of range",
    ],
)
def test_soft_limit_rejections_are_normal_boundaries(detail: str) -> None:
    assert control._benign_motion_rejection(RuntimeError(detail)) is not None


def test_connection_bound_lease_rejection_is_automatically_recoverable() -> None:
    assert control._is_recoverable_authority_error(
        RuntimeError("request requires a connection-bound lease")
    )


def test_stream_deadline_is_normal_safe_stop_feedback() -> None:
    assert control._benign_motion_rejection(
        RuntimeError("MOTION_LOOP_DEADLINE_EXPIRED")
    ) is not None


@pytest.mark.parametrize(
    "detail",
    [
        "arm rejected: ILLEGAL_TRANSITION",
        "arm grant is absent",
        "request requires a live control session",
        "STALE_COMMAND_GENERATION",
        "CAPABILITY_DIGEST_STALE",
    ],
)
def test_session_state_rejections_are_automatically_recoverable(detail: str) -> None:
    assert control._is_recoverable_authority_error(RuntimeError(detail))


@pytest.mark.parametrize("detail_code", ["OVERSHOOT", "ERROR_GROWTH"])
def test_confirmed_stop_control_deviation_is_nonfatal_feedback(
    detail_code: str,
) -> None:
    assert control._benign_motion_rejection(
        RuntimeError(
            f"motion terminal outcome=FAILED detail_code={detail_code} "
            "stop_outcome=CONFIRMED_STOPPED"
        )
    ) is not None
    assert control._benign_motion_rejection(
        RuntimeError(
            f"motion terminal outcome=RESULT_UNKNOWN detail_code={detail_code} "
            "stop_outcome=SENT_UNCONFIRMED; safe state unknown"
        )
    ) is None


def test_zdt_urdf_base_and_j4_visual_directions_are_reversed_only_in_the_scene(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
) -> None:
    monkeypatch.setenv(
        "WALDO_URDF_ANGLE_OFFSETS_PATH",
        str(tmp_path / "missing-machine-offsets.json"),
    )
    assert commander_main._urdf_angle_signs("parol6_zdt_backend") == [
        -1,
        1,
        1,
        -1,
        1,
        1,
    ]
    assert commander_main._urdf_angle_signs("parol6") == [1] * 6
    assert commander_main._urdf_angle_offsets("parol6_zdt_backend") == [0.0] * 6
    assert commander_main._urdf_angle_offsets("parol6") == [0.0] * 6


def test_jog_enablement_uses_independent_safety_state_caches(monkeypatch) -> None:
    """A joint refresh must not swallow the cartesian safety transition."""

    class Element:
        def __init__(self) -> None:
            self.class_names: set[str] = set()

        def classes(self, *, add: str | None = None, remove: str | None = None):
            if add:
                self.class_names.update(add.split())
            if remove:
                self.class_names.difference_update(remove.split())
            return self

    available = SimpleNamespace(
        can_jog_pos=[True] * 6,
        can_jog_neg=[True] * 6,
    )
    status = SimpleNamespace(
        editing_mode=False,
        simulator_active=True,
        connected=False,
        joints=SimpleNamespace(
            can_jog_pos=[True] * 6,
            can_jog_neg=[True] * 6,
        ),
        pose=SimpleNamespace(
            cart_jog=SimpleNamespace(by_frame={"WRF": available, "TRF": available})
        ),
    )
    monkeypatch.setattr(
        control.waldoctl, "commander", SimpleNamespace(status=status)
    )
    monkeypatch.setattr(control, "_read_only_mode", lambda: False)
    monkeypatch.setattr(control, "is_any_program_running", lambda: False)
    monkeypatch.setattr(
        control.ui_state,
        "robot",
        SimpleNamespace(
            joints=SimpleNamespace(count=6),
            cartesian_frames=("WRF", "TRF"),
        ),
    )

    panel = object.__new__(control.ControlPanel)
    panel._n_joints = 6
    panel._joint_left_btns = {i: Element() for i in range(6)}
    panel._joint_right_btns = {i: Element() for i in range(6)}
    panel._cart_axis_imgs = {axis: Element() for axis in control._AXIS_ORDER}
    panel._cart_slot_elems = {}
    panel._last_joint_pos = None
    panel._last_joint_neg = None
    panel._last_cart_wrf_pos = None
    panel._last_cart_wrf_neg = None
    panel._last_cart_trf_pos = None
    panel._last_cart_trf_neg = None
    panel._last_joint_controls_available = None
    panel._last_cart_controls_available = None

    panel.refresh_joint_enablement()
    panel.sync_cartesian_button_states()
    assert all(
        "cp-disabled-strong" not in elem.class_names
        for elem in panel._cart_axis_imgs.values()
    )

    # The live status adapter mutates direction arrays in place. Value-based
    # caching must still observe the change and refresh the affected buttons.
    status.joints.can_jog_pos[0] = False
    available.can_jog_pos[0] = False
    panel.refresh_joint_enablement()
    panel.sync_cartesian_button_states()
    assert "cp-disabled-strong" in panel._joint_right_btns[0].class_names
    assert "cp-disabled-strong" in panel._cart_axis_imgs["X+"].class_names
    status.joints.can_jog_pos[0] = True
    available.can_jog_pos[0] = True
    panel.refresh_joint_enablement()
    panel.sync_cartesian_button_states()
    assert "cp-disabled-strong" not in panel._joint_right_btns[0].class_names
    assert "cp-disabled-strong" not in panel._cart_axis_imgs["X+"].class_names

    # These calls run in this exact order in the status consumer. Before the
    # regression fix, the joint call updated a shared editing-mode cache and
    # the cartesian call returned early with stale enabled visuals.
    status.editing_mode = True
    panel.refresh_joint_enablement()
    panel.sync_cartesian_button_states()
    assert all(
        "cp-disabled-strong" in elem.class_names
        for elem in panel._joint_left_btns.values()
    )
    assert all(
        "cp-disabled-strong" in elem.class_names
        for elem in panel._cart_axis_imgs.values()
    )

    # A silent hardware disconnect must also fail closed even when the
    # backend's per-direction arrays have not changed.
    status.editing_mode = False
    status.simulator_active = False
    status.connected = False
    panel.refresh_joint_enablement()
    panel.sync_cartesian_button_states()
    assert all(
        "cp-disabled-strong" in elem.class_names
        for elem in panel._joint_right_btns.values()
    )
    assert all(
        "cp-disabled-strong" in elem.class_names
        for elem in panel._cart_axis_imgs.values()
    )


@pytest.mark.asyncio
async def test_incremental_moves_reject_overlapping_clicks() -> None:
    panel = object.__new__(control.ControlPanel)
    panel._incremental_move_lock = asyncio.Lock()
    started = asyncio.Event()
    release = asyncio.Event()
    calls = 0

    async def operation() -> None:
        nonlocal calls
        calls += 1
        started.set()
        await release.wait()

    first = asyncio.create_task(panel._run_incremental_move("joint", operation))
    await started.wait()
    second = asyncio.create_task(panel._run_incremental_move("joint", operation))
    await asyncio.sleep(0)

    assert calls == 1
    await second
    assert calls == 1

    release.set()
    await first
    assert calls == 1


@pytest.mark.asyncio
async def test_unreachable_incremental_move_is_non_fault_feedback(monkeypatch) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._incremental_move_lock = asyncio.Lock()

    class UiClient:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

    panel._ui_client = UiClient()
    notifications: list[tuple[str, str]] = []
    monkeypatch.setattr(
        control.ui,
        "notify",
        lambda message, *, color, **_kwargs: notifications.append((message, color)),
    )

    async def operation() -> None:
        raise RuntimeError("IK target is unreachable")

    await panel._run_incremental_move("cart", operation)

    assert notifications == [
        (
            "该方向当前不可达，机械臂未执行并保持停止。请反向移动或调整姿态。",
            "info",
        )
    ]


@pytest.mark.asyncio
async def test_incremental_move_auto_recovers_incomplete_authority(monkeypatch) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._incremental_move_lock = asyncio.Lock()
    panel._ui_client = None
    calls: list[str] = []
    notifications: list[tuple[str, str]] = []

    async def stop() -> int:
        calls.append("stop")
        return 1

    async def reset() -> int:
        calls.append("reset")
        return 1

    attempts = 0

    async def operation() -> None:
        nonlocal attempts
        attempts += 1
        calls.append(f"move-{attempts}")
        if attempts == 1:
            raise RuntimeError("current grant and lease are required")

    panel.client = SimpleNamespace(stop=stop, reset=reset)
    monkeypatch.setattr(
        control.ui,
        "notify",
        lambda message, *, color, **_kwargs: notifications.append((message, color)),
    )

    await panel._run_incremental_move("joint", operation)

    assert calls == ["move-1", "stop", "reset", "move-2"]
    assert notifications == [
        ("控制授权已自动恢复，本次动作已重新执行。", "positive")
    ]


@pytest.mark.parametrize(
    ("raw_error", "expected_reason", "expected_solution"),
    [
        (
            "CAPABILITY_NOT_AUTHORIZED",
            "目标超出本次授权的运动范围",
            "请减小步长或使用分段移动",
        ),
        (
            "terminal target error 0.44 deg exceeds 0.25 deg",
            "机械臂已安全停止，但实际位置与目标存在偏差",
            "系统不会自动纠偏或锁定控制",
        ),
        (
            "overlapping official motion is not allowed",
            "上一动作尚未完成",
            "请等待按钮恢复后再操作",
        ),
        (
            "simulator methods are FakeCAN-only",
            "当前 SocketCAN 真机后端不支持页面仿真模式",
            "请保持真机模式",
        ),
        (
            "Robot mode requires a hardware connection",
            "机械臂硬件尚未连接",
            "请确认 worker、can0 和适配器在线",
        ),
        (
            "COMMAND_TIMEOUT",
            "动作等待超时，系统已执行停止",
            "请降低速度或步长",
        ),
        (
            "stable terminal encoder sampling deadline expired",
            "终点编码器稳定采样未在规定时间内完成",
            "请点击页面上的“恢复控制”",
        ),
        (
            "current grant and lease are required",
            "内部控制授权未完整建立",
            "自动停止、回收旧授权并重新开放操作",
        ),
        (
            "STOP_ALREADY_PENDING",
            "上一条停止命令仍在收尾",
            "等待停止完成并自动复位",
        ),
        (
            "OFFICIAL_TRAJECTORY_INVALID: Joint 2 target (-148.4 deg) is out of range",
            "目标超出当前关节允许范围",
            "页面应自动禁用该方向",
        ),
    ],
)
def test_operator_errors_are_chinese_and_actionable(
    raw_error: str,
    expected_reason: str,
    expected_solution: str,
) -> None:
    message = operator_error("关节动作", raw_error)
    assert expected_reason in message
    assert expected_solution in message
    assert raw_error not in message
    assert "处理方法：" in message


@pytest.mark.asyncio
async def test_small_safe_terminal_miss_is_displayed_without_retry(
    monkeypatch,
) -> None:
    panel = object.__new__(control.ControlPanel)
    calls: list[tuple[list[float], float, float]] = []
    notifications: list[tuple[str, str | None]] = []
    monkeypatch.setattr(
        control.ui,
        "notify",
        lambda message, *, color=None, **_kwargs: notifications.append((message, color)),
    )

    async def move_j(target, *, speed, accel, wait, timeout):
        calls.append((list(target), speed, accel))
        assert wait is True
        assert timeout == 120.0
        raise RuntimeError(
            "outcome=RESULT_UNKNOWN detail_code=OFFICIAL_MOTION_RUNTIME_REJECTED "
            "stop_outcome=CONFIRMED_STOPPED detail=SAFE_TERMINAL "
            "J1 terminal target error 0.550397 deg exceeds 0.500000 deg"
        )

    async def angles():
        return [9.449603, 0.0, 0.0, 0.0, 0.0, 0.0]

    panel.client = SimpleNamespace(move_j=move_j, angles=angles)
    target = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    await panel._move_j_with_terminal_correction(
        target,
        joint_index=0,
        speed=1.0,
        accel=1.0,
        timeout=120.0,
    )

    assert calls == [(target, 1.0, 1.0)]
    assert notifications == [
        ("J1 已安全停止，目标偏差 +0.550°；未自动纠偏，可继续操作或手动再次移动。", "warning")
    ]


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "error",
    [
        "overlapping official motion is not allowed",
        (
            "stop_outcome=SENT_UNCONFIRMED detail=SAFE_TERMINAL "
            "J1 terminal target error 0.55 deg exceeds 0.50 deg"
        ),
        (
            "stop_outcome=CONFIRMED_STOPPED detail=SAFE_TERMINAL "
            "J1 terminal target error 1.10 deg exceeds 0.50 deg"
        ),
    ],
)
async def test_terminal_correction_rejects_unsafe_or_unrelated_failures(error) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._get_joint_limits = lambda _joint: (-180.0, 180.0)
    calls = 0

    async def move_j(*_args, **_kwargs):
        nonlocal calls
        calls += 1
        raise RuntimeError(error)

    async def angles():
        raise AssertionError("unsafe failures must not read for a retry")

    panel.client = SimpleNamespace(move_j=move_j, angles=angles)
    with pytest.raises(RuntimeError, match=error):
        await panel._move_j_with_terminal_correction(
            [0.0] * 6,
            joint_index=0,
            speed=1.0,
            accel=1.0,
            timeout=120.0,
        )
    assert calls == 1


@pytest.mark.asyncio
async def test_zdt_backend_opens_isolated_simulator_before_client_call(
    monkeypatch,
) -> None:
    panel = object.__new__(control.ControlPanel)

    async def simulator(_enabled):
        raise AssertionError("ZDT must not call the FakeCAN-only method")

    panel.client = SimpleNamespace(simulator=simulator)
    monkeypatch.setattr(
        control.ui_state,
        "robot",
        SimpleNamespace(backend_package="parol6_zdt_backend"),
    )
    scripts: list[str] = []

    async def run_javascript(script):
        scripts.append(script)

    monkeypatch.setattr(
        control.ui,
        "run_javascript",
        run_javascript,
    )

    await panel.on_toggle_sim()

    assert len(scripts) == 1
    assert ":8012/" in scripts[0]
    assert "window.open(" in scripts[0]
    assert "_blank" in scripts[0]


def test_non_loopback_primary_browser_is_distinct_from_local_automation() -> None:
    def page(host: str):
        return SimpleNamespace(
            request=SimpleNamespace(client=SimpleNamespace(host=host))
        )

    assert commander_main._client_is_loopback(page("127.0.0.1")) is True
    assert commander_main._client_is_loopback(page("::1")) is True
    assert commander_main._client_is_loopback(page("192.168.1.5")) is False


def test_browser_holder_description_includes_device_browser_and_tab() -> None:
    page = SimpleNamespace(
        id="client-id",
        tab_id="tab-123456789",
        request=SimpleNamespace(
            client=SimpleNamespace(host="192.168.1.88"),
            headers={"user-agent": "Mozilla/5.0 Chrome/140.0"},
        ),
    )

    assert commander_main._browser_client_description(page) == (
        "192.168.1.88 · Chrome · 标签页 tab-1234"
    )


def test_takeover_token_is_host_bound_short_lived_and_single_use(monkeypatch) -> None:
    now = 100.0
    monkeypatch.setattr(commander_main.time, "monotonic", lambda: now)
    commander_main._pending_takeovers.clear()

    def page(host: str):
        return SimpleNamespace(
            request=SimpleNamespace(client=SimpleNamespace(host=host))
        )

    source = page("192.168.1.186")
    token = commander_main._issue_takeover_token(source)

    assert commander_main._consume_takeover_token(token, page("192.168.1.5")) is False
    assert commander_main._consume_takeover_token(token, source) is False

    token = commander_main._issue_takeover_token(source)
    assert commander_main._consume_takeover_token(token, source) is True
    assert commander_main._consume_takeover_token(token, source) is False

    token = commander_main._issue_takeover_token(source)
    now += commander_main._TAKEOVER_TOKEN_TTL_S + 0.1
    assert commander_main._consume_takeover_token(token, source) is False


def test_only_real_browser_clients_can_reserve_primary_slot(monkeypatch) -> None:
    monkeypatch.delitem(commander_main.sys.modules, "pytest", raising=False)

    def page(headers: dict[str, str]):
        return SimpleNamespace(request=SimpleNamespace(headers=headers))

    chrome_headers = {
        "user-agent": "Mozilla/5.0 Chrome/140.0",
        "sec-fetch-mode": "navigate",
        "sec-fetch-dest": "document",
    }
    assert commander_main._client_is_browser_navigation(page(chrome_headers)) is True
    assert (
        commander_main._client_is_browser_navigation(
            page({"user-agent": "curl/8.10.1"})
        )
        is False
    )
    assert (
        commander_main._client_is_browser_navigation(
            page({"user-agent": "Mozilla/5.0 Chrome/140.0"})
        )
        is True
    )


@pytest.mark.asyncio
async def test_status_consumer_reconnects_after_transient_failure(monkeypatch) -> None:
    attempts = 0
    reconnected = asyncio.Event()

    async def consume_once() -> None:
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise TimeoutError("transient IPC timeout")
        reconnected.set()
        await asyncio.Future()

    monkeypatch.setattr(commander_main, "_status_consumer_once", consume_once)
    monkeypatch.setattr(commander_main, "_shutting_down", False)

    task = asyncio.create_task(commander_main._status_consumer())
    try:
        await asyncio.wait_for(reconnected.wait(), timeout=1.0)
        assert attempts == 2
    finally:
        task.cancel()
        await task


def test_joint_limit_waypoints_never_exceed_receipt_delta() -> None:
    points = control._joint_limit_waypoints(-132.0, 180.0)
    assert points == [-42.0, 48.0, 138.0, 180.0]
    segments = [b - a for a, b in zip([-132.0, *points], points)]
    assert max(abs(delta) for delta in segments) <= 90.0


@pytest.mark.asyncio
async def test_operator_stop_during_admission_is_not_reported_as_motion_failure(
    monkeypatch,
) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._incremental_move_lock = asyncio.Lock()
    panel._ui_client = None
    notifications: list[tuple[str, str]] = []
    monkeypatch.setattr(
        control.ui,
        "notify",
        lambda message, *, color, **_kwargs: notifications.append((message, color)),
    )

    async def operation() -> None:
        raise RuntimeError("official motion returned before admission")

    await panel._run_incremental_move("joint", operation)

    assert notifications == []


def test_joint_direction_availability_matches_backend_status(monkeypatch) -> None:
    panel = object.__new__(control.ControlPanel)
    monkeypatch.setattr(
        control.waldoctl,
        "commander",
        SimpleNamespace(
            status=SimpleNamespace(
                joints=SimpleNamespace(
                    can_jog_pos=[False, True, True, True, True, True],
                    can_jog_neg=[True] * 6,
                )
            )
        ),
    )

    assert panel._joint_direction_available(0, "pos") is False
    assert panel._joint_direction_available(0, "neg") is True
    assert panel._joint_direction_available(8, "pos") is False


@pytest.mark.asyncio
async def test_exact_joint_moves_share_incremental_move_lock(monkeypatch) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._incremental_move_lock = asyncio.Lock()
    panel._n_joints = 6
    panel.client = type("Client", (), {})()
    panel.client.move_j = None
    panel._movement_allowed = lambda: True
    panel._get_joint_limits = lambda _joint: (-180.0, 180.0)

    monkeypatch.setattr(
        control.waldoctl,
        "commander",
        SimpleNamespace(
            status=SimpleNamespace(
                joints=SimpleNamespace(angles=SimpleNamespace(deg=[0.0] * 6))
            )
        ),
    )
    monkeypatch.setattr(control, "_norm_speed", lambda: 0.6)
    monkeypatch.setattr(control, "_norm_accel", lambda: 0.4)

    started = asyncio.Event()
    release = asyncio.Event()
    targets: list[list[float]] = []

    refreshes = 0

    async def move_j(
        target: list[float], *, speed: float, accel: float, wait: bool, timeout: float
    ) -> None:
        assert wait is True
        assert timeout == panel.EXACT_MOVE_TIMEOUT_S
        assert accel == 0.4
        targets.append(target)
        started.set()
        await release.wait()

    async def angles() -> list[float]:
        nonlocal refreshes
        refreshes += 1
        return targets[-1]

    panel.client.move_j = move_j
    panel.client.angles = angles
    first = asyncio.create_task(panel.move_joint_to_angle(0, 1.0))
    await started.wait()
    second = asyncio.create_task(panel.move_joint_to_angle(0, 2.0))
    await asyncio.sleep(0)
    release.set()
    await asyncio.gather(first, second)

    assert targets == [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    assert refreshes == 1


@pytest.mark.asyncio
async def test_cartesian_click_waits_for_safe_terminal_and_forwards_settings(
    monkeypatch,
) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._incremental_move_lock = asyncio.Lock()
    panel._movement_allowed = lambda **_kwargs: True
    panel._cart_pressed_axes = {axis: False for axis in control._AXIS_ORDER}
    panel._cart_axis_imgs = {}
    panel._apply_pressed_style = lambda *_args: None
    panel._set_strong_disabled = lambda *_args: None
    panel._finish_jog_release = lambda *_args: None
    panel.INCREMENTAL_MOVE_TIMEOUT_S = 30.0
    panel.client = SimpleNamespace()

    class ClickHandler:
        async def on_change(self, _key, is_pressed, *, on_click, **_kwargs):
            if not is_pressed:
                await on_click()

    panel._cart_click_hold = ClickHandler()
    panel._ui_client = None
    calls: list[dict] = []

    async def move_l(pose, **kwargs):
        calls.append({"pose": pose, **kwargs})

    panel.client.move_l = move_l
    monkeypatch.setattr(control, "_norm_speed", lambda: 0.8)
    monkeypatch.setattr(control, "_norm_accel", lambda: 0.4)
    monkeypatch.setattr(
        control.waldoctl,
        "commander",
        SimpleNamespace(
            status=SimpleNamespace(
                editing_mode=False,
                pose=SimpleNamespace(cart_jog=SimpleNamespace(by_frame={})),
            ),
            settings=SimpleNamespace(jog=SimpleNamespace(joint_step_deg=2.0)),
        ),
    )
    monkeypatch.setattr(
        control.ui_state,
        "robot",
        SimpleNamespace(cartesian_frames=["WRF", "TRF"]),
    )
    monkeypatch.setattr(
        control.ui_state, "_cart_jog_timer", SimpleNamespace(active=False)
    )
    monkeypatch.setattr(control.motion_recorder, "on_jog_start", lambda *_args: None)

    await panel.set_axis_pressed("X+", False)

    assert calls == [
        {
            "pose": [2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "frame": "WRF",
            "speed": 0.8,
            "accel": 0.4,
            "rel": True,
            "wait": True,
            "timeout": 30.0,
        }
    ]


@pytest.mark.asyncio
async def test_calibration_home_success_notification_uses_page_client_context(monkeypatch) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._incremental_move_lock = asyncio.Lock()
    panel._movement_allowed = lambda **_kwargs: True
    panel.client = SimpleNamespace()
    context_active = False

    class PageClient:
        def __enter__(self):
            nonlocal context_active
            context_active = True

        def __exit__(self, *_args):
            nonlocal context_active
            context_active = False

    panel._ui_client = PageClient()

    async def move_j(*_args, **_kwargs):
        return 1

    panel.client.move_j = move_j
    monkeypatch.setattr(control, "_norm_speed", lambda: 0.1)
    monkeypatch.setattr(control, "_norm_accel", lambda: 0.1)
    notifications: list[str] = []

    def notify(message: str, **_kwargs) -> None:
        assert context_active is True
        notifications.append(message)

    monkeypatch.setattr(control.ui, "notify", notify)

    await panel._execute_calibration_home_move()

    assert notifications == ["机械臂已到达校准 Home。"]


@pytest.mark.asyncio
async def test_cartesian_jog_failure_releases_button_and_notifies(monkeypatch) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._movement_allowed = lambda **_kwargs: True
    panel._cart_pressed_axes = {axis: axis == "X+" for axis in control._AXIS_ORDER}
    panel._cart_axis_imgs = {"X+": object()}
    panel._tcp_drag_active = False
    panel._tcp_latest_pose = None
    panel.STREAM_TIMEOUT_S = 0.1
    panel.JOG_TICK_S = 0.02
    panel.CADENCE_WARN_WINDOW = 10
    panel.CADENCE_TOLERANCE = 0.1
    panel._get_first_pressed_axis = lambda: "X+"
    panel._get_cart_axis_lookup = lambda: {"X+": ("X", 1.0, "WRF")}
    panel._apply_pressed_style = lambda *_args: None
    panel._cart_cadence = SimpleNamespace(tick=lambda *_args: None)

    async def jog_l(*_args, **_kwargs):
        raise RuntimeError("current grant and lease are required")

    panel.client = SimpleNamespace(jog_l=jog_l)
    timer = SimpleNamespace(active=True)
    monkeypatch.setattr(control.ui_state, "_cart_jog_timer", timer)
    monkeypatch.setattr(control, "_norm_speed", lambda: 0.5)
    monkeypatch.setattr(control, "_norm_accel", lambda: 0.5)
    notifications: list[tuple[str, str]] = []
    monkeypatch.setattr(
        control.ui,
        "notify",
        lambda message, *, color, **_kwargs: notifications.append((message, color)),
    )

    await panel.cart_jog_tick()

    assert panel._cart_pressed_axes["X+"] is False
    assert timer.active is False
    assert notifications == [
        (
            "笛卡尔点动失败：内部控制授权未完整建立。"
            "处理方法：请点击“恢复控制”，页面会自动停止、回收旧授权并重新开放操作。",
            "negative",
        )
    ]


def test_tcp_drag_starts_from_current_pose_to_avoid_zero_motion(monkeypatch) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._movement_allowed = lambda **_kwargs: True
    panel._tcp_drag_active = False
    panel._tcp_last_sent_pose = None
    panel._cart_cadence = SimpleNamespace(reset=lambda: None)
    monkeypatch.setattr(
        control.ui_state, "_cart_jog_timer", SimpleNamespace(active=False)
    )
    monkeypatch.setattr(
        control.waldoctl,
        "commander",
        SimpleNamespace(
            status=SimpleNamespace(
                pose=SimpleNamespace(x=1.0, y=2.0, z=3.0, rx=4.0, ry=5.0, rz=6.0)
            )
        ),
    )
    monkeypatch.setattr(control.motion_recorder, "on_jog_start", lambda *_args: None)

    panel._handle_tcp_cartesian_move_start()

    assert panel._tcp_last_sent_pose == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_playback_cleanup_cancels_page_timer() -> None:
    playback = PlaybackController()
    timer = SimpleNamespace(cancelled=False, with_current_invocation=False)

    def cancel(*, with_current_invocation: bool = False) -> None:
        timer.cancelled = True
        timer.with_current_invocation = with_current_invocation

    timer.cancel = cancel
    playback._sim_timer = timer

    playback.cleanup()

    assert timer.cancelled is True
    assert timer.with_current_invocation is True
    assert playback._sim_timer is None


def test_page_cleanup_cancels_timers_before_client_is_replaced(monkeypatch) -> None:
    from waldo_commander import main

    class Timer:
        def __init__(self) -> None:
            self.cancelled = False
            self.with_current_invocation = False

        def cancel(self, *, with_current_invocation: bool = False) -> None:
            self.cancelled = True
            self.with_current_invocation = with_current_invocation

    class Panel:
        def __init__(self) -> None:
            self.cleaned = False

        def cleanup(self) -> None:
            self.cleaned = True

    page_client = object()
    ping_timer = Timer()
    scene_init_timer = Timer()
    joint_timer = Timer()
    cart_timer = Timer()
    control_panel = Panel()
    editor_panel = Panel()
    gripper_panel = Panel()

    class Scene:
        def __init__(self) -> None:
            self.cleaned = False

        def cleanup(self) -> None:
            self.cleaned = True

    scene = Scene()

    monkeypatch.setattr(
        main,
        "_page_state",
        SimpleNamespace(
            page_client=page_client,
            ping_timer=ping_timer,
            scene_init_timer=scene_init_timer,
            urdf_scene=scene,
        ),
    )
    monkeypatch.setattr(main, "control_panel", control_panel)
    monkeypatch.setattr(main, "editor_panel", editor_panel)
    monkeypatch.setattr(main.ui_state, "_joint_jog_timer", joint_timer)
    monkeypatch.setattr(main.ui_state, "_cart_jog_timer", cart_timer)
    monkeypatch.setattr(main.ui_state, "gripper_page", gripper_panel)
    monkeypatch.setattr(main.ui_state, "urdf_scene", scene)

    main._cleanup_page_resources(page_client)

    assert ping_timer.cancelled is True
    assert ping_timer.with_current_invocation is True
    assert scene_init_timer.cancelled is True
    assert scene_init_timer.with_current_invocation is True
    assert joint_timer.cancelled is True
    assert joint_timer.with_current_invocation is True
    assert cart_timer.cancelled is True
    assert cart_timer.with_current_invocation is True
    assert control_panel.cleaned is True
    assert editor_panel.cleaned is True
    assert gripper_panel.cleaned is True
    assert scene.cleaned is True
    assert main.ui_state.urdf_scene is None
    assert main._page_state is None


def test_shadow_page_cleanup_cancels_its_ping_timer() -> None:
    from waldo_commander import main

    timer = SimpleNamespace(cancelled=False, with_current_invocation=False)

    def cancel(*, with_current_invocation: bool = False) -> None:
        timer.cancelled = True
        timer.with_current_invocation = with_current_invocation

    timer.cancel = cancel
    page_client = SimpleNamespace(_waldo_shadow_ping_timer=timer)

    main._cleanup_shadow_page_timer(page_client)

    assert timer.cancelled is True
    assert timer.with_current_invocation is True
    assert page_client._waldo_shadow_ping_timer is None
