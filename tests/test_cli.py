import argparse
from pathlib import Path

from xlerobot_mcp.camera import MockCameraProvider
from xlerobot_mcp.cli import _format_startup_message
from xlerobot_mcp.controllers import MockServoController


def test_startup_message_for_stdio_right_bus_only(tmp_path: Path) -> None:
    controller = MockServoController(
        right_arm_wheel_usb="/dev/ttyUSB0",
        position_dir=tmp_path,
    )
    args = argparse.Namespace(
        name="XLeRobot Servo MCP",
        transport="stdio",
        backend="hardware",
        host="127.0.0.1",
        port=8765,
        streamable_http_path="/mcp",
        right_arm_wheel_usb="/dev/ttyUSB0",
        left_arm_head_usb=None,
        camera_index_or_path=None,
        camera_mock_image=None,
    )

    message = _format_startup_message(args, controller, MockCameraProvider())

    assert "ready on stdio" in message
    assert "right_arm_wheel_usb=/dev/ttyUSB0" in message
    assert "left_arm_head_usb=unset" in message
    assert "camera_source=mock" in message
    assert "enabled=wheels,head,right_arm,left_arm" in message


def test_startup_message_for_streamable_http(tmp_path: Path) -> None:
    controller = MockServoController(position_dir=tmp_path)
    args = argparse.Namespace(
        name="XLeRobot Servo MCP",
        transport="streamable-http",
        backend="mock",
        host="127.0.0.1",
        port=8876,
        streamable_http_path="/mcp",
        right_arm_wheel_usb=None,
        left_arm_head_usb=None,
        camera_index_or_path=None,
        camera_mock_image=None,
    )

    message = _format_startup_message(args, controller, MockCameraProvider())

    assert "http://127.0.0.1:8876/mcp" in message
    assert "transport=streamable-http" in message
