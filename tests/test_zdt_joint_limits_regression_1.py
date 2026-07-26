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

    assert panel._get_joint_limits(1) == (-101.25, -2.0625)
    assert panel._get_joint_limits(2) == (81.210937625, 228.86718770833332)
    assert panel._get_joint_limits(3) == (-28.125, 28.125)
    assert panel._get_joint_limits(4) == (-37.5, 23.203125)
    assert panel._get_joint_limits(5) == (-2.8125, 182.8125)
