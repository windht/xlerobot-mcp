"""FastMCP server for XLeRobot servo controls."""

from __future__ import annotations

import base64
import json
from typing import Any, Dict, Literal

from mcp import types
from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations

from .camera import CameraProvider, NavigationMode, UnavailableCameraProvider
from .controllers import (
    ArmSide,
    HardwareServoController,
    MockServoController,
    TorqueTarget,
)

Controller = MockServoController | HardwareServoController

SERVER_INSTRUCTIONS = (
    "This MCP server controls XLeRobot wheel, head, and arm servos. "
    "Use get_camera_image to capture the current scene before motion when visual context matters. "
    "Prefer one atomic action at a time and inspect state before repeated motion."
)


def _text_result(summary: str, structured: Dict[str, Any], *, is_error: bool = False) -> types.CallToolResult:
    return types.CallToolResult(
        content=[types.TextContent(type="text", text=summary)],
        structuredContent=structured,
        isError=is_error,
    )


def _action_result(
    controller: Controller,
    summary: str,
    *,
    camera_provider: CameraProvider,
) -> types.CallToolResult:
    return _text_result(
        summary,
        {
            "action": controller.get_last_action_report(),
            "robot_state": controller.get_state_snapshot(),
            "camera_status": camera_provider.get_status(),
        },
    )


def _handle_controller_error(exc: Exception) -> types.CallToolResult:
    return _text_result(
        str(exc),
        {"error": {"type": exc.__class__.__name__, "message": str(exc)}},
        is_error=True,
    )


def _camera_result(
    controller: Controller,
    camera_provider: CameraProvider,
    *,
    camera_fov_deg: float,
    center_angle_deg: float,
    navigation_mode: NavigationMode,
    include_debug_overlay: bool,
) -> types.CallToolResult:
    snapshot = camera_provider.capture_image(
        camera_fov_deg=camera_fov_deg,
        center_angle_deg=center_angle_deg,
        navigation_mode=navigation_mode,
        include_debug_overlay=include_debug_overlay,
    )
    dimensions = (
        f"{snapshot.width}x{snapshot.height}"
        if snapshot.width is not None and snapshot.height is not None
        else "unknown-size"
    )
    summary = (
        f"Captured current camera frame from {snapshot.source} "
        f"({dimensions}, mode={snapshot.navigation_mode}, "
        f"center_angle_deg={snapshot.center_angle_deg:.1f})."
    )
    return types.CallToolResult(
        content=[
            types.TextContent(type="text", text=summary),
            types.ImageContent(
                type="image",
                data=base64.b64encode(snapshot.image_bytes).decode("ascii"),
                mimeType=snapshot.mime_type,
            ),
        ],
        structuredContent={
            "camera": {
                "source": snapshot.source,
                "timestamp": snapshot.timestamp,
                "mime_type": snapshot.mime_type,
                "width": snapshot.width,
                "height": snapshot.height,
                "camera_fov_deg": snapshot.camera_fov_deg,
                "center_angle_deg": snapshot.center_angle_deg,
                "navigation_mode": snapshot.navigation_mode,
                "overlay_applied": snapshot.overlay_applied,
            },
            "camera_status": camera_provider.get_status(),
            "robot_state": controller.get_state_snapshot(),
        },
    )


def _require_positive(value: float, field_name: str) -> None:
    if float(value) <= 0:
        raise ValueError(f"{field_name} must be greater than 0.")


def build_server(
    controller: Controller,
    camera_provider: CameraProvider | None = None,
    *,
    name: str = "XLeRobot Servo MCP",
    host: str = "127.0.0.1",
    port: int = 8765,
    streamable_http_path: str = "/mcp",
    json_response: bool = True,
    stateless_http: bool = True,
) -> FastMCP:
    camera_provider = camera_provider or UnavailableCameraProvider()
    mcp = FastMCP(
        name=name,
        instructions=SERVER_INSTRUCTIONS,
        host=host,
        port=port,
        streamable_http_path=streamable_http_path,
        json_response=json_response,
        stateless_http=stateless_http,
    )

    @mcp.resource(
        "robot://state",
        name="robot_state",
        description="Current structured XLeRobot servo state.",
        mime_type="application/json",
    )
    def robot_state_resource() -> str:
        return json.dumps(controller.get_state_snapshot(), indent=2)

    @mcp.tool(
        title="Get Robot State",
        description="Return structured controller state, pose estimate, head angles, arm positions, and torque state.",
        annotations=ToolAnnotations(readOnlyHint=True, idempotentHint=True),
    )
    def get_robot_state() -> types.CallToolResult:
        snapshot = controller.get_state_snapshot()
        capabilities = snapshot["capabilities"]
        summary = (
            f"Backend: {snapshot['backend']}. "
            f"Wheels={capabilities['wheels']}, head={capabilities['head']}, "
            f"left_arm={capabilities['left_arm']}, right_arm={capabilities['right_arm']}. "
            f"Camera configured={camera_provider.get_status()['configured']}."
        )
        return _text_result(
            summary,
            {
                "robot_state": snapshot,
                "camera_status": camera_provider.get_status(),
            },
        )

    @mcp.tool(
        title="Get Camera Image",
        description=(
            "Capture the current camera picture for visual understanding. "
            "Returns a text summary, an image payload, and structured camera metadata."
        ),
        annotations=ToolAnnotations(readOnlyHint=True),
    )
    def get_camera_image(
        camera_fov_deg: float = 120.0,
        center_angle_deg: float = 0.0,
        navigation_mode: Literal["normal", "precision"] = "normal",
        include_debug_overlay: bool = True,
    ) -> types.CallToolResult:
        try:
            return _camera_result(
                controller,
                camera_provider,
                camera_fov_deg=float(camera_fov_deg),
                center_angle_deg=float(center_angle_deg),
                navigation_mode=navigation_mode,
                include_debug_overlay=include_debug_overlay,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Move Forward",
        description="Move the wheel base forward by the requested distance in meters.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def move_forward(meters: float) -> types.CallToolResult:
        try:
            _require_positive(meters, "meters")
            controller.go_forward(meters)
            return _action_result(
                controller,
                f"Moved forward {float(meters):.3f} m.",
                camera_provider=camera_provider,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Move Backward",
        description="Move the wheel base backward by the requested distance in meters.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def move_backward(meters: float) -> types.CallToolResult:
        try:
            _require_positive(meters, "meters")
            controller.go_backward(meters)
            return _action_result(
                controller,
                f"Moved backward {float(meters):.3f} m.",
                camera_provider=camera_provider,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Turn Left",
        description="Rotate the wheel base left by the requested number of degrees.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def turn_left(degrees: float) -> types.CallToolResult:
        try:
            _require_positive(degrees, "degrees")
            controller.turn_left(degrees)
            return _action_result(
                controller,
                f"Turned left {float(degrees):.3f} deg.",
                camera_provider=camera_provider,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Turn Right",
        description="Rotate the wheel base right by the requested number of degrees.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def turn_right(degrees: float) -> types.CallToolResult:
        try:
            _require_positive(degrees, "degrees")
            controller.turn_right(degrees)
            return _action_result(
                controller,
                f"Turned right {float(degrees):.3f} deg.",
                camera_provider=camera_provider,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Strafe Left",
        description="Strafe the wheel base left by the requested distance in meters.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def strafe_left(meters: float) -> types.CallToolResult:
        try:
            _require_positive(meters, "meters")
            controller.strafe_left(meters)
            return _action_result(
                controller,
                f"Strafed left {float(meters):.3f} m.",
                camera_provider=camera_provider,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Strafe Right",
        description="Strafe the wheel base right by the requested distance in meters.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def strafe_right(meters: float) -> types.CallToolResult:
        try:
            _require_positive(meters, "meters")
            controller.strafe_right(meters)
            return _action_result(
                controller,
                f"Strafed right {float(meters):.3f} m.",
                camera_provider=camera_provider,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Stop Wheels",
        description="Send zero velocity to the wheel motors immediately.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def stop_wheels() -> types.CallToolResult:
        try:
            controller.stop_wheels()
            return _action_result(
                controller,
                "Stopped wheel motion.",
                camera_provider=camera_provider,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Turn Head Yaw",
        description="Set the head yaw servo angle in degrees. Values are clamped to the robot's safe range.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def turn_head_yaw(degrees: float) -> types.CallToolResult:
        try:
            payload = controller.turn_head_yaw(degrees)
            applied = next(iter(payload.values()))
            return _action_result(
                controller,
                f"Set head yaw to {float(applied):.3f} deg.",
                camera_provider=camera_provider,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Turn Head Pitch",
        description="Set the head pitch servo angle in degrees. Values are clamped to the robot's safe range.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def turn_head_pitch(degrees: float) -> types.CallToolResult:
        try:
            payload = controller.turn_head_pitch(degrees)
            applied = next(iter(payload.values()))
            return _action_result(
                controller,
                f"Set head pitch to {float(applied):.3f} deg.",
                camera_provider=camera_provider,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Turn Head To VLA Position",
        description="Move the head into the RoboCrew VLA-friendly default pose.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def turn_head_to_vla_position(pitch_deg: float = 45.0) -> types.CallToolResult:
        try:
            controller.turn_head_to_vla_position(pitch_deg)
            return _action_result(
                controller,
                "Moved head to the VLA position.",
                camera_provider=camera_provider,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Reset Head Position",
        description="Reset the head to the RoboCrew neutral pose.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def reset_head_position() -> types.CallToolResult:
        try:
            controller.reset_head_position()
            return _action_result(
                controller,
                "Reset head position.",
                camera_provider=camera_provider,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Set Arm Position",
        description="Set one or both arm poses. Positions must be a JSON object keyed by joint name.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def set_arm_position(
        positions: dict[str, float],
        arm_side: ArmSide = "both",
    ) -> types.CallToolResult:
        try:
            applied = controller.set_arm_position(positions, arm_side)
            return _text_result(
                f"Updated {arm_side} arm position.",
                {
                    "action": controller.get_last_action_report(),
                    "applied_positions": dict(applied),
                    "robot_state": controller.get_state_snapshot(),
                    "camera_status": camera_provider.get_status(),
                },
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Read Arm Present Position",
        description="Read the current arm joint positions without issuing motion.",
        annotations=ToolAnnotations(readOnlyHint=True),
    )
    def read_arm_present_position(arm_side: ArmSide = "both") -> types.CallToolResult:
        try:
            positions = controller.read_arm_present_position(arm_side)
            return _text_result(
                f"Read {arm_side} arm position.",
                {"arm_side": arm_side, "positions": dict(positions)},
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Save Arm Position",
        description="Save the current arm pose to a named JSON preset for later reuse.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def save_arm_position(
        position_name: str = "arm_positions",
        arm_side: ArmSide = "both",
    ) -> types.CallToolResult:
        try:
            path = controller.save_arm_position(position_name, arm_side)
            return _text_result(
                f"Saved {arm_side} arm pose as '{position_name}'.",
                {
                    "action": controller.get_last_action_report(),
                    "path": path,
                    "robot_state": controller.get_state_snapshot(),
                    "camera_status": camera_provider.get_status(),
                },
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Set Saved Position",
        description="Load a named saved arm pose and apply it to the requested arm side.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def set_saved_position(
        position_name: str,
        arm_side: ArmSide = "both",
    ) -> types.CallToolResult:
        try:
            positions = controller.set_saved_position(position_name, arm_side)
            return _text_result(
                f"Applied saved pose '{position_name}' to {arm_side}.",
                {
                    "action": controller.get_last_action_report(),
                    "applied_positions": dict(positions),
                    "robot_state": controller.get_state_snapshot(),
                    "camera_status": camera_provider.get_status(),
                },
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Enable Torque",
        description="Enable servo torque for all motors or a target group.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def enable_torque(target: TorqueTarget = "all") -> types.CallToolResult:
        try:
            controller.enable_torque(target)
            return _action_result(
                controller,
                f"Enabled torque for {target}.",
                camera_provider=camera_provider,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    @mcp.tool(
        title="Disable Torque",
        description="Disable servo torque for all motors or a target group.",
        annotations=ToolAnnotations(destructiveHint=True),
    )
    def disable_torque(target: TorqueTarget = "all") -> types.CallToolResult:
        try:
            controller.disable_torque(target)
            return _action_result(
                controller,
                f"Disabled torque for {target}.",
                camera_provider=camera_provider,
            )
        except Exception as exc:
            return _handle_controller_error(exc)

    return mcp
