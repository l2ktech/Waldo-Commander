from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest
from waldo_commander.components import control
from waldo_commander import main as commander_main
from waldo_commander.components.playback import PlaybackController
from waldo_commander.operator_messages import operator_error


def test_incremental_joint_moves_allow_slow_hardware_settling() -> None:
    assert control.ControlPanel.INCREMENTAL_MOVE_TIMEOUT_S == 120.0
    assert control.ControlPanel.EXACT_MOVE_TIMEOUT_S == 120.0


def test_zdt_speed_rating_spans_the_encodable_range() -> None:
    assert control._normalized_speed(10, "parol6_zdt_backend") == pytest.approx(0.1)
    assert control._normalized_speed(50, "parol6_zdt_backend") == pytest.approx(0.5)
    assert control._normalized_speed(100, "parol6_zdt_backend") == pytest.approx(1.0)


def test_zdt_urdf_base_visual_direction_is_reversed_only_in_the_scene() -> None:
    assert commander_main._urdf_angle_signs("parol6_zdt_backend") == [
        -1,
        1,
        1,
        1,
        1,
        1,
    ]
    assert commander_main._urdf_angle_signs("parol6") == [1] * 6


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
async def test_incremental_move_failure_is_visible_to_the_operator(monkeypatch) -> None:
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
            "笛卡尔动作失败：操作未完成，系统已保留诊断信息。"
            "处理方法：请先确认页面状态正常后重试；"
            "若再次失败，请查看服务日志或联系维护人员。",
            "negative",
        )
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
            "机械臂已停止，但终点误差超过验收范围",
            "请降低速度或加速度后重试",
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
            "笛卡尔点动失败：页面控制授权已失效。"
            "处理方法：请等待上一动作结束；仍未恢复时刷新页面并重新接管。",
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
