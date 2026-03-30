"""CLI entrypoint for the XLeRobot servo MCP server."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from .camera import CameraProvider, build_camera_provider
from .controllers import (
    DEFAULT_ARM_POSITION_DIR,
    DEFAULT_SPEED,
    ConfigurationError,
    ControllerError,
    HardwareServoController,
    MockServoController,
)
from .server import build_server


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Expose XLeRobot servo controls through MCP.",
    )
    parser.add_argument(
        "--backend",
        choices=("mock", "hardware"),
        default=os.getenv("XLEROBOT_BACKEND", "mock"),
        help="Controller backend to use. Defaults to mock.",
    )
    parser.add_argument(
        "--right-arm-wheel-usb",
        default=os.getenv("XLEROBOT_RIGHT_ARM_WHEEL_USB"),
        help="USB path for the right-arm + wheel bus. This is enough for initial hardware testing.",
    )
    parser.add_argument(
        "--left-arm-head-usb",
        default=os.getenv("XLEROBOT_LEFT_ARM_HEAD_USB"),
        help="Optional USB path for the left-arm + head bus.",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=int(os.getenv("XLEROBOT_SPEED", str(DEFAULT_SPEED))),
        help="Base wheel speed multiplier.",
    )
    parser.add_argument(
        "--position-dir",
        default=os.getenv("XLEROBOT_POSITION_DIR", DEFAULT_ARM_POSITION_DIR),
        help="Directory for saved arm pose JSON files.",
    )
    parser.add_argument(
        "--camera-index-or-path",
        default=os.getenv("XLEROBOT_CAMERA_INDEX_OR_PATH"),
        help="Live camera source for visual observation, for example 0 or /dev/video0.",
    )
    parser.add_argument(
        "--camera-width",
        type=int,
        default=int(os.getenv("XLEROBOT_CAMERA_WIDTH", "640")),
        help="Requested camera width for live capture.",
    )
    parser.add_argument(
        "--camera-height",
        type=int,
        default=int(os.getenv("XLEROBOT_CAMERA_HEIGHT", "480")),
        help="Requested camera height for live capture.",
    )
    parser.add_argument(
        "--camera-fps",
        type=int,
        default=int(os.getenv("XLEROBOT_CAMERA_FPS", "30")),
        help="Requested camera FPS for live capture.",
    )
    parser.add_argument(
        "--camera-mock-image",
        default=os.getenv("XLEROBOT_CAMERA_MOCK_IMAGE"),
        help="Optional static image file to expose through get_camera_image when a live camera is not configured.",
    )
    parser.add_argument(
        "--transport",
        choices=("stdio", "streamable-http", "sse"),
        default=os.getenv("XLEROBOT_TRANSPORT", "stdio"),
        help="MCP transport to run.",
    )
    parser.add_argument(
        "--host",
        default=os.getenv("XLEROBOT_HOST", "127.0.0.1"),
        help="Host for HTTP-based transports.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.getenv("XLEROBOT_PORT", "8765")),
        help="Port for HTTP-based transports.",
    )
    parser.add_argument(
        "--streamable-http-path",
        default=os.getenv("XLEROBOT_STREAMABLE_HTTP_PATH", "/mcp"),
        help="Path for streamable HTTP transport.",
    )
    parser.add_argument(
        "--json-response",
        action=argparse.BooleanOptionalAction,
        default=_env_bool("XLEROBOT_JSON_RESPONSE", True),
        help="Use JSON responses for streamable HTTP.",
    )
    parser.add_argument(
        "--stateless-http",
        action=argparse.BooleanOptionalAction,
        default=_env_bool("XLEROBOT_STATELESS_HTTP", True),
        help="Serve stateless HTTP responses when using streamable HTTP.",
    )
    parser.add_argument(
        "--name",
        default=os.getenv("XLEROBOT_SERVER_NAME", "XLeRobot Servo MCP"),
        help="Display name announced to MCP clients.",
    )
    return parser


def _build_controller(args: argparse.Namespace) -> MockServoController | HardwareServoController:
    common = {
        "right_arm_wheel_usb": args.right_arm_wheel_usb,
        "left_arm_head_usb": args.left_arm_head_usb,
        "speed": args.speed,
        "position_dir": Path(args.position_dir).expanduser(),
    }
    if args.backend == "hardware":
        return HardwareServoController(**common)
    return MockServoController(**common)


def _build_camera_provider(args: argparse.Namespace) -> CameraProvider:
    return build_camera_provider(
        camera_index_or_path=args.camera_index_or_path,
        camera_width=args.camera_width,
        camera_height=args.camera_height,
        camera_fps=args.camera_fps,
        camera_mock_image=args.camera_mock_image,
    )


def _format_startup_message(
    args: argparse.Namespace,
    controller: MockServoController | HardwareServoController,
    camera_provider: CameraProvider,
) -> str:
    state = controller.get_state_snapshot()
    capabilities = state["capabilities"]
    enabled = [name for name, enabled in capabilities.items() if enabled]
    disabled = [name for name, enabled in capabilities.items() if not enabled]
    camera_status = camera_provider.get_status()

    if args.transport == "stdio":
        location = "stdio"
    elif args.transport == "streamable-http":
        location = f"http://{args.host}:{args.port}{args.streamable_http_path}"
    else:
        location = f"http://{args.host}:{args.port}/sse"

    message = (
        f"{args.name} ready on {location} "
        f"(transport={args.transport}, backend={args.backend}, "
        f"right_arm_wheel_usb={args.right_arm_wheel_usb or 'unset'}, "
        f"left_arm_head_usb={args.left_arm_head_usb or 'unset'}, "
        f"camera_source={camera_status.get('source') or 'unset'}, "
        f"enabled={','.join(enabled) or 'none'}"
    )
    if disabled:
        message += f", disabled={','.join(disabled)}"
    message += ")"
    return message


def _emit_status(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    try:
        controller = _build_controller(args)
        camera_provider = _build_camera_provider(args)
        server = build_server(
            controller,
            camera_provider,
            name=args.name,
            host=args.host,
            port=args.port,
            streamable_http_path=args.streamable_http_path,
            json_response=args.json_response,
            stateless_http=args.stateless_http,
        )
        try:
            _emit_status(_format_startup_message(args, controller, camera_provider))
            server.run(args.transport)
        finally:
            camera_provider.release()
            controller.disconnect()
            _emit_status(f"{args.name} stopped.")
    except (ControllerError, ConfigurationError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
