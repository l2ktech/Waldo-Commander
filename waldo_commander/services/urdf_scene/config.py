"""Configuration dataclasses for UrdfScene."""

import json
import math
import os
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Callable, Sequence

from waldo_commander.common.theme import SceneColors


def _angle_offsets_path() -> Path:
    """Resolve the machine-local visual calibration path at call time."""
    return Path(
        os.environ.get(
            "WALDO_URDF_ANGLE_OFFSETS_PATH",
            "/var/lib/parol6-zdt/waldo/urdf-angle-offsets.json",
        )
    ).expanduser()


def load_angle_offsets(defaults: list[float]) -> list[float]:
    """Load visual-only URDF offsets, falling back to installed defaults."""
    try:
        payload = json.loads(_angle_offsets_path().read_text(encoding="utf-8"))
        values = payload.get("angle_offsets_deg")
        if (
            payload.get("schema") != "waldo-urdf-angle-offsets-v1"
            or not isinstance(values, list)
            or len(values) != len(defaults)
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                for value in values
            )
        ):
            return list(defaults)
        return [float(value) for value in values]
    except (OSError, ValueError, TypeError):
        return list(defaults)


def save_angle_offsets(values: list[float]) -> None:
    """Persist visual-only offsets without changing robot calibration."""
    normalized = [float(value) for value in values]
    if len(normalized) != 6 or any(not math.isfinite(value) for value in normalized):
        raise ValueError("visual angle offsets must contain six finite values")
    angle_offsets_path = _angle_offsets_path()
    angle_offsets_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = angle_offsets_path.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(
            {
                "schema": "waldo-urdf-angle-offsets-v1",
                "angle_offsets_deg": normalized,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(angle_offsets_path)


class RobotAppearanceMode(Enum):
    """Robot visual appearance modes.

    LIVE: Normal robot view showing real-time joint angles from robot_state
    SIMULATOR: Amber/ghost appearance, still shows real-time angles
    EDITING: Grey semi-transparent appearance for target editing, shows editing angles
    """

    LIVE = "live"
    SIMULATOR = "simulator"
    EDITING = "editing"


@dataclass
class ToolPose:
    """TCP offset and orientation for a tool."""

    origin: Sequence[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: Sequence[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class UrdfSceneConfig:
    """Configuration for UrdfScene behavior, appearance, and kinematics."""

    meshes_dir: Path | None = None
    """Directory containing mesh files. If None, auto-discover from URDF location."""

    static_url_prefix: str = "/meshes"
    """URL prefix for serving static mesh files."""

    package_map: dict[str, Path] = field(default_factory=lambda: {})
    """Mapping from package:// names to filesystem paths."""

    mount_static: bool = True
    """Whether to automatically mount meshes as static files."""

    scale_stls: float = 1.0
    """Scale factor for all STL files (e.g., 1e-1 if designed in mm)."""

    gizmo_scale: float | None = None
    """Override gizmo size. If None, scales with STL scale."""

    draw_tcp_axes: bool = True
    """Whether to draw coordinate axes at TCP location."""

    tool_pose_map: dict[str, "ToolPose"] = field(default_factory=lambda: {})
    """Mapping from tool names to TCP poses."""

    tool_pose_resolver: Callable[[str, str | None], "ToolPose | None"] | None = None
    """Function to resolve tool name to TCP pose dynamically.

    Signature: ``(tool_key, variant_key) -> ToolPose | None``.
    """

    # Colors from theme.py SceneColors
    material: str = SceneColors.MATERIAL_DARK_HEX
    """Default material color for robot meshes."""

    background_color: str = SceneColors.BACKGROUND_DARK_HEX
    """Scene background color."""

    ground_color: str = SceneColors.GROUND_DARK_HEX
    """Ground plane color (contrasts with background)."""

    sim_color: str = SceneColors.SIM_AMBER_HEX
    """Color for robot in simulator mode (amber ghost)."""

    sim_opacity: float = 0.9
    """Opacity for robot in simulator mode."""

    edit_color: str = SceneColors.EDIT_GRAY_HEX
    """Color for robot in editing mode (grey ghost)."""

    edit_opacity: float = 0.4
    """Opacity for robot in editing mode."""

    tool_body_material: str = SceneColors.TOOL_BODY_HEX
    """Color for tool body meshes in live mode."""

    tool_body_sim_color: str = SceneColors.TOOL_BODY_SIM_HEX
    """Color for tool body meshes in simulator mode."""

    tool_body_edit_color: str = SceneColors.TOOL_BODY_EDIT_HEX
    """Color for tool body meshes in editing mode."""

    tool_moving_material: str = SceneColors.TOOL_MOVING_HEX
    """Color for tool moving parts in live mode."""

    tool_moving_sim_color: str = SceneColors.TOOL_MOVING_SIM_HEX
    """Color for tool moving parts in simulator mode."""

    tool_moving_edit_color: str = SceneColors.TOOL_MOVING_EDIT_HEX
    """Color for tool moving parts in editing mode."""

    joint_name_order: list[str] = field(
        default_factory=lambda: ["L1", "L2", "L3", "L4", "L5", "L6"]
    )
    """Order of joint names for mapping controller angles to URDF joints."""

    deg_to_rad: bool = True
    """Whether to convert angles from degrees to radians."""

    angle_signs: list[int] = field(default_factory=lambda: [1, 1, 1, 1, 1, 1])
    """Sign corrections for each joint angle."""

    angle_offsets: list[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    """Offset corrections for each joint angle (in degrees if deg_to_rad is True)."""
