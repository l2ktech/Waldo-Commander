from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest
from waldo_commander.components import control


def test_incremental_joint_moves_allow_slow_hardware_settling() -> None:
    assert control.ControlPanel.INCREMENTAL_MOVE_TIMEOUT_S == 30.0
    assert control.ControlPanel.EXACT_MOVE_TIMEOUT_S == 30.0


def test_zdt_speed_is_clamped_to_encodable_floor() -> None:
    assert control._normalized_speed(1, "parol6_zdt_backend") == 0.6


@pytest.mark.asyncio
async def test_incremental_moves_coalesce_overlapping_clicks() -> None:
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
    await panel._run_incremental_move("joint", operation)
    release.set()
    await first

    assert calls == 1


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

    async def move_j(target: list[float], *, speed: float, timeout: float) -> None:
        assert timeout == 30.0
        targets.append(target)
        started.set()
        await release.wait()

    panel.client.move_j = move_j
    first = asyncio.create_task(panel.move_joint_to_angle(0, 1.0))
    await started.wait()
    await panel.move_joint_to_angle(0, 2.0)
    release.set()
    await first

    assert targets == [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
