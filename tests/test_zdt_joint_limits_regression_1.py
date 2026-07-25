"""Regression coverage for deployed ZDT joint-limit UI behavior."""

from types import SimpleNamespace

from waldo_commander.components import control


def test_zdt_joint_controls_use_installed_hardware_limits(monkeypatch) -> None:
    # Regression: QA-001 — J2 negative was disabled at -148.42° even though
    # the installed receipt permits motion down to -151.875°.
    # Found by /qa on 2026-07-25.
    # Report: .gstack/qa-reports/qa-report-192-168-1-5-2026-07-25.md
    monkeypatch.setattr(
        control.ui_state,
        "robot",
        SimpleNamespace(backend_package="parol6_zdt_backend"),
    )
    panel = object.__new__(control.ControlPanel)

    assert panel._get_joint_limits(1) == (-151.875, -3.09375)
    assert panel._get_joint_limits(4) == (-150.0, 92.8125)
    assert panel._get_joint_limits(5) == (-2.8125, 182.8125)
