"""Effective robot limits shared by controls and 3D workspace rendering."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import numpy as np


ZDT_JOINT_LIMITS_DEG = np.asarray(
    (
        (-117.4758195, 137.4069925),
        (-137.0, -37.098302649122786),
        (96.179468625, 243.83571870833333),
        (-23.121469, 33.128531),
        (-90.0, 3.163844),
        (7.174012142857137, 192.79901214285712),
    ),
    dtype=np.float64,
)

ZDT_JOINT_LIMITS_PATH = Path(
    os.environ.get(
        "WALDO_ZDT_JOINT_LIMITS_PATH",
        "/var/lib/parol6-zdt/waldo/joint-limits.json",
    )
)


def _installed_zdt_joint_limits_deg() -> np.ndarray:
    """Load the limit file generated with the active hardware receipt."""
    try:
        payload = json.loads(ZDT_JOINT_LIMITS_PATH.read_text(encoding="utf-8"))
        if payload.get("schema") != "parol6-zdt/joint-limits/v1":
            raise ValueError("unexpected joint-limit schema")
        limits = np.asarray(payload["joint_limits_deg"], dtype=np.float64)
        if limits.shape != (6, 2) or not np.isfinite(limits).all():
            raise ValueError("joint limits must be a finite 6x2 matrix")
        if np.any(limits[:, 0] >= limits[:, 1]):
            raise ValueError("joint-limit minimum must be below maximum")
        return limits
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        return ZDT_JOINT_LIMITS_DEG


def effective_joint_limits_deg(robot: Any) -> np.ndarray:
    """Return the deployed soft limits used by both motion controls and FK hulls."""
    if getattr(robot, "backend_package", "") == "parol6_zdt_backend":
        return _installed_zdt_joint_limits_deg().copy()
    return np.asarray(robot.joints.limits.position.deg, dtype=np.float64).copy()


def effective_joint_limits_rad(robot: Any) -> np.ndarray:
    """Return :func:`effective_joint_limits_deg` converted to radians."""
    return np.deg2rad(effective_joint_limits_deg(robot))
