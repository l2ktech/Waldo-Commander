"""Effective robot limits shared by controls and 3D workspace rendering."""

from __future__ import annotations

from typing import Any

import numpy as np


ZDT_JOINT_LIMITS_DEG = np.asarray(
    (
        (-117.4758195, 137.4069925),
        (-137.0, -37.098302649122786),
        (96.179468625, 243.83571870833333),
        (-23.121469, 33.128531),
        (-57.539281, 3.163844),
        (7.174012142857137, 192.79901214285712),
    ),
    dtype=np.float64,
)


def effective_joint_limits_deg(robot: Any) -> np.ndarray:
    """Return the deployed soft limits used by both motion controls and FK hulls."""
    if getattr(robot, "backend_package", "") == "parol6_zdt_backend":
        return ZDT_JOINT_LIMITS_DEG.copy()
    return np.asarray(robot.joints.limits.position.deg, dtype=np.float64).copy()


def effective_joint_limits_rad(robot: Any) -> np.ndarray:
    """Return :func:`effective_joint_limits_deg` converted to radians."""
    return np.deg2rad(effective_joint_limits_deg(robot))
