"""MCP tools for selecting and directly operating the active robot tool."""

from __future__ import annotations

import waldoctl

from waldo_commander.mcp.server import get_mcp
from waldo_commander.mcp.tools.control import require_actuation, require_control

mcp = get_mcp()


def _accepted(index: int, operation: str) -> dict[str, object]:
    if index < 0:
        raise RuntimeError(f"tool.{operation} was not accepted by the controller")
    return {"accepted": True, "command_index": index}


@mcp.tool(name="tool.select")
async def select(key: str = "STS3215", variant_key: str = "") -> dict[str, object]:
    """Select a configured tool. Selection changes software state but does not move it."""
    require_control()
    normalized = key.strip().upper()
    result = await waldoctl.commander.client.select_tool(normalized, variant_key)
    response = _accepted(result, "select")
    response["tool"] = normalized
    return response


async def _action(
    action: str,
    params: list[object] | None = None,
    *,
    wait: bool = True,
    timeout: float = 10.0,
) -> dict[str, object]:
    result = await waldoctl.commander.client.tool_action(
        "STS3215",
        action,
        params,
        wait=wait,
        timeout=timeout,
    )
    response = _accepted(result, action)
    response["tool"] = "STS3215"
    response["action"] = action
    return response


@mcp.tool(name="tool.open")
async def open_tool(timeout: float = 10.0) -> dict[str, object]:
    """Open the selected STS3215 gripper."""
    require_actuation("open STS3215 gripper")
    return await _action("open", timeout=timeout)


@mcp.tool(name="tool.close")
async def close_tool(timeout: float = 10.0) -> dict[str, object]:
    """Close the selected STS3215 gripper."""
    require_actuation("close STS3215 gripper")
    return await _action("close", timeout=timeout)


@mcp.tool(name="tool.move")
async def move(
    position: float,
    speed: float | None = None,
    current: int | None = None,
    wait: bool = True,
    timeout: float = 10.0,
) -> dict[str, object]:
    """Move STS3215 to normalized closure 0..1 with optional speed/current limits."""
    require_actuation(f"move STS3215 gripper to {position}")
    params: list[object] = [position]
    if speed is not None or current is not None:
        params.append(1.0 if speed is None else speed)
    if current is not None:
        params.append(current)
    return await _action("move", params, wait=wait, timeout=timeout)


@mcp.tool(name="tool.stop")
async def stop(timeout: float = 10.0) -> dict[str, object]:
    """Stop STS3215 motion. Stopping is deliberately ungated."""
    return await _action("stop", timeout=timeout)
