"""Smoke tests for Waldo Commander app startup and basic UI presence."""

import pytest
import numpy as np
from nicegui.testing import User

from tests.helpers.wait import wait_for_app_ready


def test_new_scene_applies_cached_backend_angles(monkeypatch) -> None:
    """A new URDF scene must not render one frame at the zero-angle pose."""
    import waldoctl
    import waldo_commander.main as subject

    expected = np.array([8.3, -132.8, 109.7, -24.0, -63.3, 76.1])
    waldoctl.commander.status.joints.angles.set_deg(expected)
    applied: list[tuple[object, np.ndarray]] = []

    class Scene:
        def __init__(self) -> None:
            self.updated = False

        def update_from_robot_state(self) -> None:
            self.updated = True

    scene = Scene()
    monkeypatch.setattr(
        subject,
        "update_urdf_angles",
        lambda angles, target_scene: applied.append(
            (target_scene, np.array(angles, copy=True))
        ),
    )

    subject._sync_scene_to_cached_status(scene)

    assert len(applied) == 1
    assert applied[0][0] is scene
    np.testing.assert_allclose(applied[0][1], expected)
    assert scene.updated is True


def test_two_takeovers_route_real_angles_only_to_current_page(monkeypatch) -> None:
    """A late scene from either evicted page must never replace the active scene."""
    import waldo_commander.main as subject
    from waldo_commander.state import ui_state

    class PageClient:
        def __init__(self, client_id: str) -> None:
            self.id = client_id

    class Scene:
        def __init__(self, name: str) -> None:
            self.name = name
            self.robot_state_updates = 0

        def update_from_robot_state(self) -> None:
            self.robot_state_updates += 1

    reset_calls: list[None] = []
    angle_targets: list[Scene] = []
    monkeypatch.setattr(
        subject, "reset_angle_pipeline", lambda: reset_calls.append(None)
    )
    monkeypatch.setattr(
        subject,
        "update_urdf_angles",
        lambda _angles, target_scene: angle_targets.append(target_scene),
    )

    first = subject._PageState(page_client=PageClient("first"))
    second = subject._PageState(page_client=PageClient("second"))
    current = subject._PageState(page_client=PageClient("current"))
    first_scene = Scene("first")
    second_scene = Scene("second")
    current_scene = Scene("current")
    late_first_scene = Scene("late-first")
    late_second_scene = Scene("late-second")

    for page, scene in (
        (first, first_scene),
        (second, second_scene),
        (current, current_scene),
    ):
        monkeypatch.setattr(subject, "_page_state", page)
        monkeypatch.setattr(ui_state, "active_client_id", page.page_client.id)
        assert subject._activate_page_scene(page, scene) is True

    assert subject._activate_page_scene(first, late_first_scene) is False
    assert subject._activate_page_scene(second, late_second_scene) is False

    subject._update_page_scene_from_status(current)

    assert ui_state.urdf_scene is current_scene
    assert angle_targets == [current_scene]
    assert current_scene.robot_state_updates == 1
    assert first_scene.robot_state_updates == 0
    assert second_scene.robot_state_updates == 0
    assert len(reset_calls) == 3


def test_takeover_overlay_offers_explicit_control_recovery() -> None:
    import inspect
    import waldo_commander.main as subject

    source = inspect.getsource(subject._build_takeover_overlay)

    assert "接管控制" in source
    assert "control_lease.release(BROWSER, held_client.id)" in source
    assert "_issue_takeover_token(c)" in source
    assert "window.location.replace('/?takeover=" in source


def test_read_only_mode_rejects_motion_without_backend_call(monkeypatch) -> None:
    from waldo_commander.components.control import ControlPanel

    monkeypatch.setenv("WALDO_READ_ONLY", "1")
    assert ControlPanel._movement_allowed(notify=False) is False


@pytest.mark.integration
async def test_status_consumer_starts(user: User) -> None:
    """Status consumer must start and receive data from the controller.

    Regression test: the server readiness check used loop.sock_sendto()
    which is not implemented by uvloop (NiceGUI's event loop).  This caused
    start_controller() to fail silently, leaving the status consumer
    uncreated and the UI frozen with stale position data.
    """
    from waldo_commander.state import readiness_state

    await user.open("/")
    await wait_for_app_ready(timeout_s=15.0)
    assert readiness_state._backend_done, (
        "status consumer never received a STATUS update"
    )


@pytest.mark.integration
async def test_root_page_loads(user: User) -> None:
    """Test that the root page loads successfully and returns HTTP 200.

    This is a basic smoke test to ensure the app starts without errors.
    """
    await user.open("/")
    # User fixture automatically asserts HTTP 200


@pytest.mark.integration
async def test_core_ui_markers_present(user: User) -> None:
    """Test that core UI elements are present on the main page.

    Verifies that key control panel buttons, tabs, and readout elements
    are rendered and visible using their marker attributes.
    """
    await user.open("/")

    # Control panel buttons
    await user.should_see(marker="btn-home")
    await user.should_see(marker="btn-home-coordinate-restore")
    await user.should_see(marker="btn-pre-grasp")
    await user.should_see(marker="btn-robot-toggle")
    await user.should_see(marker="btn-estop")

    # Side tabs
    await user.should_see(marker="tab-program")
    await user.should_see(marker="tab-io")
    await user.should_see(marker="tab-settings")
    await user.should_see(marker="tab-gripper")

    # Readout panel (at least one coordinate)
    await user.should_see(marker="readout-x")


@pytest.mark.integration
async def test_joint_jog_buttons_present(user: User) -> None:
    """Test that joint jog buttons are rendered for all joints.

    Verifies that the joint control interface is properly built.
    """
    await user.open("/")

    # Check that at least J1 plus and minus buttons exist
    await user.should_see(marker="btn-j1-plus")
    await user.should_see(marker="btn-j1-minus")

    # Check that J6 exists (last joint)
    await user.should_see(marker="btn-j6-plus")
    await user.should_see(marker="btn-j6-minus")
