from pathlib import Path

from xlerobot_mcp.camera import MockCameraProvider
from xlerobot_mcp.controllers import MockServoController
from xlerobot_mcp.server import build_server


def test_mock_camera_provider_returns_image_snapshot() -> None:
    provider = MockCameraProvider()

    snapshot = provider.capture_image(
        camera_fov_deg=90.0,
        center_angle_deg=12.0,
        navigation_mode="precision",
    )

    assert snapshot.mime_type == "image/png"
    assert snapshot.image_bytes
    assert snapshot.navigation_mode == "precision"
    assert snapshot.center_angle_deg == 12.0
    assert snapshot.camera_fov_deg == 90.0


def test_server_registers_get_camera_image_tool(tmp_path: Path) -> None:
    server = build_server(MockServoController(position_dir=tmp_path), MockCameraProvider())

    tool_names = [tool.name for tool in server._tool_manager.list_tools()]

    assert "get_camera_image" in tool_names
