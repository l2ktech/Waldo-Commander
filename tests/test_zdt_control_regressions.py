from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest
from waldo_commander.components import control
from waldo_commander.components.playback import PlaybackController


def test_incremental_joint_moves_allow_slow_hardware_settling() -> None:
    assert control.ControlPanel.INCREMENTAL_MOVE_TIMEOUT_S == 30.0
    assert control.ControlPanel.EXACT_MOVE_TIMEOUT_S == 30.0


def test_zdt_speed_is_clamped_to_encodable_floor() -> None:
    assert control._normalized_speed(1, "parol6_zdt_backend") == 0.6


@pytest.mark.asyncio
async def test_incremental_moves_queue_overlapping_clicks() -> None:
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

    release.set()
    await asyncio.gather(first, second)

    assert calls == 2


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

    started = asyncio.Event()
    release = asyncio.Event()
    targets: list[list[float]] = []

    refreshes = 0

    async def move_j(
        target: list[float], *, speed: float, wait: bool, timeout: float
    ) -> None:
        assert wait is True
        assert timeout == 30.0
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

    assert targets == [
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    assert refreshes == 2


def test_playback_cleanup_cancels_page_timer() -> None:
    playback = PlaybackController()
    timer = SimpleNamespace(cancelled=False)
    timer.cancel = lambda: setattr(timer, "cancelled", True)
    playback._sim_timer = timer

    playback.cleanup()

    assert timer.cancelled is True
    assert playback._sim_timer is None


def test_page_cleanup_cancels_timers_before_client_is_replaced(monkeypatch) -> None:
    from waldo_commander import main

    class Timer:
        def __init__(self) -> None:
            self.cancelled = False

        def cancel(self) -> None:
            self.cancelled = True

    class Panel:
        def __init__(self) -> None:
            self.cleaned = False

        def cleanup(self) -> None:
            self.cleaned = True

    page_client = object()
    ping_timer = Timer()
    joint_timer = Timer()
    cart_timer = Timer()
    control_panel = Panel()
    editor_panel = Panel()
    gripper_panel = Panel()

    monkeypatch.setattr(
        main,
        "_page_state",
        SimpleNamespace(page_client=page_client, ping_timer=ping_timer),
    )
    monkeypatch.setattr(main, "control_panel", control_panel)
    monkeypatch.setattr(main, "editor_panel", editor_panel)
    monkeypatch.setattr(main.ui_state, "_joint_jog_timer", joint_timer)
    monkeypatch.setattr(main.ui_state, "_cart_jog_timer", cart_timer)
    monkeypatch.setattr(main.ui_state, "gripper_page", gripper_panel)

    main._cleanup_page_resources(page_client)

    assert ping_timer.cancelled is True
    assert joint_timer.cancelled is True
    assert cart_timer.cancelled is True
    assert control_panel.cleaned is True
    assert editor_panel.cleaned is True
    assert gripper_panel.cleaned is True
    assert main._page_state is None
