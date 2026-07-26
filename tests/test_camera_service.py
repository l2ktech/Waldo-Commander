"""Tests for the camera service MJPEG streaming and backend selection."""

from __future__ import annotations

from unittest.mock import patch

import pytest

from waldo_commander.services.camera_service import (
    CameraService,
    LinuxpyBackend,
    OpenCVBackend,
    _BLACK_1PX,
    _enumerate_v4l2_cli,
)

# A minimal valid JPEG (1x1 white pixel).
_SAMPLE_JPEG = bytes.fromhex(
    "ffd8ffe000104a46494600010100000100010000"
    "ffdb004300080606070605080707070909080a0c"
    "140d0c0b0b0c1912130f141d1a1f1e1d1a1c1c"
    "20242e2720222c231c1c2837292c30313434341f"
    "27393d38323c2e333432ffc0000b080001000101"
    "011100ffc4001f000001050101010101010000000"
    "0000000000102030405060708090a0bffc4001f01"
    "0003010101010101010101000000000000010203"
    "0405060708090a0bffda00080101000003f00000"
    "ffd9"
)


@pytest.mark.unit
def test_get_latest_frame_returns_placeholder_then_cached():
    """get_latest_frame returns placeholder initially, then cached JPEG."""
    cs = CameraService()
    assert cs.get_latest_frame() == _BLACK_1PX
    assert not cs.active

    cs._latest_jpeg = _SAMPLE_JPEG
    assert cs.get_latest_frame() == _SAMPLE_JPEG

    cs.stop()
    assert cs.get_latest_frame() == _BLACK_1PX


@pytest.mark.unit
def test_backend_selection_prefers_linuxpy_on_linux():
    """On Linux, start() tries LinuxpyBackend first, falls back to OpenCV."""

    open_calls: list[str] = []

    class FakeLinuxpy(LinuxpyBackend):
        def open(self, device, width, height):
            open_calls.append("linuxpy")
            return True

        def read_frame(self):
            return _SAMPLE_JPEG

        def close(self):
            pass

    class FakeOpenCV(OpenCVBackend):
        def open(self, device, width, height):
            open_calls.append("opencv")
            return True

        def read_frame(self):
            return _SAMPLE_JPEG

        def close(self):
            pass

    cs = CameraService()

    with (
        patch("waldo_commander.services.camera_service.LinuxpyBackend", FakeLinuxpy),
        patch("waldo_commander.services.camera_service.OpenCVBackend", FakeOpenCV),
        patch("waldo_commander.services.camera_service.sys") as mock_sys,
    ):
        mock_sys.platform = "linux"
        cs.start(0)

    assert cs.active
    assert open_calls == ["linuxpy"]
    cs.stop()


@pytest.mark.unit
def test_backend_fallback_to_opencv_when_linuxpy_fails():
    """When LinuxpyBackend.open() returns False, falls back to OpenCV."""

    open_calls: list[str] = []

    class FailLinuxpy(LinuxpyBackend):
        def open(self, device, width, height):
            open_calls.append("linuxpy")
            return False

        def close(self):
            pass

    class FakeOpenCV(OpenCVBackend):
        def open(self, device, width, height):
            open_calls.append("opencv")
            return True

        def read_frame(self):
            return _SAMPLE_JPEG

        def close(self):
            pass

    cs = CameraService()

    with (
        patch("waldo_commander.services.camera_service.LinuxpyBackend", FailLinuxpy),
        patch("waldo_commander.services.camera_service.OpenCVBackend", FakeOpenCV),
        patch("waldo_commander.services.camera_service.sys") as mock_sys,
    ):
        mock_sys.platform = "linux"
        cs.start(0)

    assert cs.active
    assert open_calls == ["linuxpy", "opencv"]
    cs.stop()


@pytest.mark.unit
def test_exclusive_capture_temporarily_releases_and_restores_camera():
    class FakeLinuxpy(LinuxpyBackend):
        def open(self, device, width, height):
            return True

        def read_frame(self):
            return _SAMPLE_JPEG

        def close(self):
            pass

    cs = CameraService()
    with (
        patch("waldo_commander.services.camera_service.LinuxpyBackend", FakeLinuxpy),
        patch("waldo_commander.services.camera_service.sys") as mock_sys,
    ):
        mock_sys.platform = "linux"
        cs.start(4, width=640, height=480)
        assert cs.suspend_for_exclusive_capture() is True
        assert not cs.active
        assert cs.resume_after_exclusive_capture() is True
        assert cs.active
    cs.stop()


@pytest.mark.unit
def test_v4l2_cli_enumeration_skips_metadata_nodes(tmp_path):
    video0 = tmp_path / "video0"
    video1 = tmp_path / "video1"
    video12 = tmp_path / "video12"
    for path in (video0, video1, video12):
        path.touch()

    def run(command, capture_output, text, check, timeout):
        del capture_output, text, check, timeout
        return type("Result", (), {"returncode": 1 if command[2].endswith("1") else 0})()

    with (
        patch("waldo_commander.services.camera_service.shutil.which", return_value="v4l2-ctl"),
        patch("waldo_commander.services.camera_service.Path", return_value=tmp_path),
        patch("waldo_commander.services.camera_service.subprocess.run", side_effect=run),
    ):
        assert _enumerate_v4l2_cli(32) == [
            {"index": 0, "label": "Camera 0"},
            {"index": 12, "label": "Camera 12"},
        ]
