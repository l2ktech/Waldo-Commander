"""Regressions for control-panel visibility and failed jog cleanup."""

from __future__ import annotations

from types import SimpleNamespace
import asyncio

import pytest

from waldo_commander.components import control
from waldo_commander.operator_messages import operator_error


@pytest.mark.asyncio
async def test_jog_timers_are_created_once_before_controls_are_used(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._ui_client = SimpleNamespace(
        __enter__=lambda self: self,
        __exit__=lambda self, *_args: None,
    )

    monkeypatch.setattr(control.ui_state, "_joint_jog_timer", None)
    monkeypatch.setattr(control.ui_state, "_cart_jog_timer", None)

    panel.ensure_jog_timers()
    joint_timer = control.ui_state._joint_jog_timer
    cart_timer = control.ui_state._cart_jog_timer
    panel.ensure_jog_timers()

    assert control.ui_state._joint_jog_timer is joint_timer
    assert control.ui_state._cart_jog_timer is cart_timer
    assert joint_timer.active is False
    assert cart_timer.active is False
    joint_timer.cancel()
    cart_timer.cancel()


@pytest.mark.asyncio
async def test_completed_incremental_move_ignores_optional_angle_refresh_failure() -> None:
    panel = object.__new__(control.ControlPanel)
    panel._n_joints = 6

    async def angles():
        raise RuntimeError("IPC client transport is not open")

    panel.client = SimpleNamespace(angles=angles)

    await panel._refresh_angles_best_effort()


@pytest.mark.asyncio
async def test_completed_incremental_move_publishes_fresh_angles_before_unlock(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._n_joints = 6
    expected = [-12.0, -99.0, 91.3, -7.5, -17.2, 61.8]

    async def angles():
        return expected

    published: list[list[float]] = []
    panel.client = SimpleNamespace(angles=angles)
    monkeypatch.setattr(
        control.waldoctl.commander,
        "status",
        SimpleNamespace(
            joints=SimpleNamespace(
                angles=SimpleNamespace(
                    set_deg=lambda values: published.append(values.tolist())
                )
            )
        ),
    )
    await panel._refresh_angles_best_effort()

    assert published == [expected]


@pytest.mark.asyncio
async def test_fault_reset_holds_motion_lock_until_stop_and_reset_finish(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import asyncio

    panel = object.__new__(control.ControlPanel)
    panel._fault_reset_action_lock = asyncio.Lock()
    panel._incremental_move_lock = asyncio.Lock()
    panel._fault_reset_btn = None
    panel._ui_client = None
    stop_entered = asyncio.Event()
    finish_stop = asyncio.Event()

    async def stop() -> int:
        stop_entered.set()
        await finish_stop.wait()
        return 1

    async def reset() -> int:
        return 1

    panel.client = SimpleNamespace(stop=stop, reset=reset)
    monkeypatch.setattr(control, "require_browser_control", lambda _cid: True)
    monkeypatch.setattr(control.ui, "notify", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(control.waldoctl.commander.status.io, "estop", 1)

    task = asyncio.create_task(panel.on_fault_reset_click())
    await stop_entered.wait()

    assert panel._incremental_move_lock.locked()
    finish_stop.set()
    await task
    assert not panel._incremental_move_lock.locked()


def test_confirmed_post_disable_settle_timeout_is_not_a_page_fault() -> None:
    error = RuntimeError(
        "motion terminal outcome=RESULT_UNKNOWN "
        "stop_outcome=CONFIRMED_STOPPED "
        "detail=post-disable mechanical stabilization deadline expired"
    )

    assert control._is_operator_stop_terminal(error) is True


@pytest.mark.parametrize(
    "error",
    (
        "safety rejected StopCompleted: AXIS_CONFIG_MISSING",
        "safety rejected arm: STOP_ALREADY_PENDING",
        "fault reset requires the current confirmed STOP proof",
        "operator stop is not confirmed",
    ),
)
def test_stale_stop_context_requests_bounded_worker_rebuild(error: str) -> None:
    assert control._requires_worker_rebuild(error)


def test_real_collision_fault_does_not_bypass_recovery_gate() -> None:
    assert not control._requires_worker_rebuild(
        "axis J5 reports a fault: stalled=True"
    )


@pytest.mark.asyncio
async def test_digital_reset_uses_bounded_rebuild_for_stale_stop_context(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    async def stop() -> int:
        raise RuntimeError("safety rejected StopCompleted: AXIS_CONFIG_MISSING")

    requested: list[bool] = []

    async def rebuild() -> None:
        requested.append(True)

    manager = control._EStopManager(SimpleNamespace(stop=stop), lambda: None)
    manager._digital_active = True
    monkeypatch.setattr(control, "require_browser_control", lambda _cid: True)
    monkeypatch.setattr(control.ui, "notify", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(control.waldoctl.commander.status.io, "estop", 1)
    monkeypatch.setattr(control, "_request_bounded_worker_rebuild", rebuild)

    await manager.reset_digital_stop()

    assert requested == [True]
    assert manager._digital_active is True


@pytest.mark.asyncio
async def test_hold_watchdog_releases_input_when_browser_mouseup_is_lost() -> None:
    handler = control._ClickHoldHandler(0.001, max_hold_s=0.01)
    events: list[object] = []

    await handler.on_change(
        "J3+",
        True,
        on_click=lambda: events.append("click"),
        on_hold_start=lambda: events.append("hold"),
        on_release=lambda was_holding: events.append(was_holding),
    )
    await asyncio.sleep(0.03)

    assert events == ["hold", True]
    assert not handler.any_active
    handler.cleanup()


def test_simplified_operator_stop_terminal_is_not_a_page_fault() -> None:
    error = RuntimeError(
        "OPERATOR_STOP_CONFIRMED: operator STOP confirmed before target completion"
    )

    assert control._is_operator_stop_terminal(error) is True


@pytest.mark.asyncio
async def test_operator_estop_remains_latched_without_red_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._incremental_move_lock = __import__("asyncio").Lock()
    panel._ui_client = None
    calls: list[str] = []

    async def stop() -> int:
        calls.append("stop")
        return 1

    async def reset() -> int:
        calls.append("reset")
        return 1

    panel.client = SimpleNamespace(stop=stop, reset=reset)

    async def operation() -> None:
        raise RuntimeError(
            "OPERATOR_STOP_CONFIRMED: operator STOP confirmed before target completion"
        )

    notifications: list[str] = []
    monkeypatch.setattr(
        control.ui,
        "notify",
        lambda message, **_kwargs: notifications.append(message),
    )

    await panel._run_incremental_move("joint", operation)

    assert calls == []
    assert notifications == []


@pytest.mark.asyncio
async def test_confirmed_safe_stop_auto_resets_without_red_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._incremental_move_lock = __import__("asyncio").Lock()
    panel._ui_client = None
    stops: list[bool] = []
    resets: list[bool] = []

    async def stop() -> int:
        stops.append(True)
        return 1

    async def reset() -> int:
        resets.append(True)
        return 1

    panel.client = SimpleNamespace(stop=stop, reset=reset)

    async def operation() -> None:
        raise RuntimeError(
            "stop_outcome=CONFIRMED_STOPPED "
            "detail=post-disable mechanical stabilization deadline expired"
        )

    notifications: list[str] = []
    monkeypatch.setattr(
        control.ui,
        "notify",
        lambda message, **_kwargs: notifications.append(message),
    )

    await panel._run_incremental_move("joint", operation)

    assert stops == [True]
    assert resets == [True]
    assert notifications == []


@pytest.mark.asyncio
async def test_joint_jog_failure_stops_timer_and_clears_all_pressed_axes(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    panel = object.__new__(control.ControlPanel)
    panel._movement_allowed = lambda **_kwargs: True
    panel._n_joints = 6
    panel._jog_pressed_pos = [False, False, False, False, False, True]
    panel._jog_pressed_neg = [False] * 6
    panel._joint_left_btns = {5: object()}
    panel._joint_right_btns = {5: object()}
    panel._apply_pressed_style = lambda *_args: None
    panel._joint_cadence = SimpleNamespace(tick=lambda *_args: None)
    panel._get_first_pressed_joint = lambda: (5, "pos")
    panel._joint_direction_available = lambda *_args: True
    panel.STREAM_TIMEOUT_S = 0.1
    panel.JOG_TICK_S = 0.02
    panel.CADENCE_WARN_WINDOW = 10
    panel.CADENCE_TOLERANCE = 0.1

    async def jog_j(*_args, **_kwargs):
        raise RuntimeError("safety rejected arm: ARM_GLOBAL_GATE_FAILED")

    panel.client = SimpleNamespace(jog_j=jog_j)
    timer = SimpleNamespace(active=True)
    monkeypatch.setattr(control.ui_state, "_joint_jog_timer", timer)
    monkeypatch.setattr(control, "_norm_speed", lambda: 0.5)
    monkeypatch.setattr(control, "_norm_accel", lambda: 0.5)
    notifications: list[str] = []
    monkeypatch.setattr(
        control.ui,
        "notify",
        lambda message, **_kwargs: notifications.append(message),
    )

    await panel.jog_tick()

    assert panel._jog_pressed_pos == [False] * 6
    assert panel._jog_pressed_neg == [False] * 6
    assert timer.active is False
    assert notifications == [
        "关节点动失败：运动准入状态暂时未恢复。"
        "处理方法：页面已停止本次连续点动；请点击“恢复”一次，"
        "系统会重新检查停止状态后开放操作。"
    ]


def test_arm_global_gate_error_has_specific_chinese_recovery() -> None:
    message = operator_error(
        "关节动作", "safety rejected arm: ARM_GLOBAL_GATE_FAILED"
    )

    assert "运动准入状态暂时未恢复" in message
    assert "点击“恢复”一次" in message
    assert "操作未完成" not in message


@pytest.mark.asyncio
async def test_sts3215_unsupported_calibration_does_not_call_backend(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[str] = []

    async def action_r(_engaged: bool) -> None:
        calls.append("calibrate")

    tool = SimpleNamespace(
        key="STS3215",
        action_r_labels=("Calibrate", "Calibrate"),
        action_r=action_r,
    )
    panel = object.__new__(control._ToolQuickActions)
    panel._movement_allowed = lambda: True
    panel._get_active_tool = lambda: tool
    notifications: list[str] = []
    monkeypatch.setattr(
        control.ui,
        "notify",
        lambda message, **_kwargs: notifications.append(message),
    )

    await panel._on_action_r()

    assert calls == []
    assert notifications == [
        "当前 STS3215 ROS2 驱动不支持页面校准；开合位置已使用现场标定值。"
    ]
