"""Regression coverage for deployed ZDT joint-limit UI behavior."""

from types import SimpleNamespace

from waldo_commander.components import control


def test_zdt_joint_controls_use_installed_hardware_limits(monkeypatch) -> None:
    # Regression: the UI must track the active axis card after a ratio update.
    # Found by /qa on 2026-07-25.
    # Report: .gstack/qa-reports/qa-report-192-168-1-5-2026-07-25.md
    monkeypatch.setattr(
        control.ui_state,
        "robot",
        SimpleNamespace(backend_package="parol6_zdt_backend"),
    )
    panel = object.__new__(control.ControlPanel)

    assert panel._get_joint_limits(0) == (-117.4758195, 137.4069925)
    assert panel._get_joint_limits(1) == (-137.0, -37.098302649122786)
    assert panel._get_joint_limits(2) == (96.179468625, 243.83571870833333)
    assert panel._get_joint_limits(3) == (-23.121469, 33.128531)
    assert panel._get_joint_limits(4) == (-90.0, 3.163844)
    assert panel._get_joint_limits(5) == (7.174012142857137, 192.79901214285712)
