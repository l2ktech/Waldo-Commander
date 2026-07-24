from __future__ import annotations

import asyncio

import pytest
from waldo_commander.components import control


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
