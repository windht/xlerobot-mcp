# XLeRobot Servo MCP Server

An MCP server that exposes the XLeRobot servo and wheel controls from RoboCrew's `servo_controls.py` to any MCP-compatible AI client.

This server stays close to the upstream `ServoControler` API, but adds:

- a standard MCP tool surface
- a `mock` backend for development without hardware
- camera observation through an MCP image tool
- a `get_robot_state` tool and `robot://state` resource for structured state
- both `stdio` and `streamable-http` transports

## TL;DR

Install `uv`, then start the mock server instantly without cloning the repo:

```bash
uvx xlerobot-mcp --backend mock
```

For Codex:

```bash
codex mcp add xlerobot -- uvx xlerobot-mcp --backend mock
```

## What It Exposes

The server wraps the same control areas as the upstream RoboCrew module:

- visual observation: `get_camera_image`
- wheel motion: `move_forward`, `move_backward`, `turn_left`, `turn_right`, `strafe_left`, `strafe_right`, `stop_wheels`
- head motion: `turn_head_yaw`, `turn_head_pitch`, `turn_head_to_vla_position`, `reset_head_position`
- arm pose control: `set_arm_position`, `read_arm_present_position`, `save_arm_position`, `set_saved_position`
- torque management: `enable_torque`, `disable_torque`
- state inspection: `get_robot_state`

## Quick Start

Use Python 3.11+.

```bash
uv sync
uv run xlerobot-mcp --backend mock
```

That starts a `stdio` MCP server in mock mode, which is the easiest way to connect from Codex, Claude Desktop, Cursor, Goose, or any other client that launches MCP servers as subprocesses.

Once the package is published to PyPI, users can skip cloning the repo and run it directly with `uvx`:

```bash
uvx xlerobot-mcp --backend mock
```

The shorter `xlerobot-mcp` command is the primary entrypoint. The original `xlerobot-servo-mcp` command remains available as a compatibility alias.

## Hardware Mode

To talk to the real robot, install the hardware dependency and pass one or both RoboCrew USB ports.

For the first round of testing, the right-arm + wheel bus is enough:

```bash
uv sync --extra hardware
uv run xlerobot-mcp \
  --backend hardware \
  --right-arm-wheel-usb /dev/ttyUSB0 \
  --camera-index-or-path 0
```

On Linux, the project now resolves PyTorch from the CPU-only index by default, so `uv sync --extra hardware` will not pull in NVIDIA CUDA packages on AMD or CPU-only machines. If you do want the default PyPI-backed Linux wheels instead, use:

```bash
uv sync --extra hardware --no-sources
```

Add the left-arm + head bus later when you're ready:

```bash
uv run xlerobot-mcp \
  --backend hardware \
  --right-arm-wheel-usb /dev/ttyUSB0 \
  --left-arm-head-usb /dev/ttyUSB1 \
  --camera-index-or-path 0
```

The hardware path is adapted from:

- [RoboCrew `servo_controls.py`](https://github.com/Grigorij-Dudnik/RoboCrew/blob/master/src/robocrew/robots/XLeRobot/servo_controls.py)

If you synced this project before the Feetech dependency was added, run `uv sync --extra hardware` again. The hardware path needs the `scservo_sdk` Python module, which is provided by the `feetech-servo-sdk` package pulled in through `lerobot[feetech]`.

## Camera Tool

RoboCrew feeds current camera frames back into the agent loop, and this server now exposes the same idea through `get_camera_image`.

For a live camera:

```bash
uv run xlerobot-mcp \
  --backend hardware \
  --right-arm-wheel-usb /dev/ttyUSB0 \
  --camera-index-or-path 0
```

For mock visual testing with a fixed image:

```bash
uv run xlerobot-mcp \
  --backend mock \
  --camera-mock-image /absolute/path/to/test-frame.jpg
```

The tool returns:

- a short text summary
- an MCP `image` payload with the current frame
- structured metadata like timestamp, source, dimensions, FOV, and navigation mode

## Streamable HTTP

If your client prefers a URL-based MCP server instead of `stdio`, run:

```bash
uv run xlerobot-mcp \
  --backend mock \
  --transport streamable-http \
  --host 127.0.0.1 \
  --port 8765
```

Then point the client at:

```text
http://127.0.0.1:8765/mcp
```

The default host is loopback-only on purpose.

## Environment Variables

CLI flags can also be provided with environment variables:

- `XLEROBOT_BACKEND=mock|hardware`
- `XLEROBOT_RIGHT_ARM_WHEEL_USB=/dev/ttyUSB0`
- `XLEROBOT_LEFT_ARM_HEAD_USB=/dev/ttyUSB1` optional
- `XLEROBOT_SPEED=10000`
- `XLEROBOT_POSITION_DIR=~/.cache/xlerobot-mcp/positions`
- `XLEROBOT_CAMERA_INDEX_OR_PATH=0`
- `XLEROBOT_CAMERA_WIDTH=640`
- `XLEROBOT_CAMERA_HEIGHT=480`
- `XLEROBOT_CAMERA_FPS=30`
- `XLEROBOT_CAMERA_MOCK_IMAGE=/absolute/path/to/test-frame.jpg`
- `XLEROBOT_TRANSPORT=stdio|streamable-http|sse`
- `XLEROBOT_HOST=127.0.0.1`
- `XLEROBOT_PORT=8765`
- `XLEROBOT_STREAMABLE_HTTP_PATH=/mcp`

## Publishing

Build the distributions the same way your users will consume them from PyPI:

```bash
uv build --no-sources
```

Then publish with a PyPI token:

```bash
uv publish
```

This repository also includes [`.github/workflows/publish.yml`](/Users/windht/Dev/xlerobot-control/.github/workflows/publish.yml), which publishes automatically when you push a version tag like `v0.1.0`.

If you need to cut another release, bump the version first, for example:

```bash
uv version patch
```

## Codex Example

After publishing:

```bash
codex mcp add xlerobot -- uvx xlerobot-mcp --backend mock
```

From a local checkout before publishing:

```bash
codex mcp add xlerobot -- \
  uv run --directory /Users/windht/Dev/xlerobot-control \
  xlerobot-mcp \
  --backend mock
```

Swap `--backend mock` for the hardware flags when you're ready to talk to the real robot.

## Notes

- `mock` mode is the default so the server is safe to start on any machine.
- In hardware mode, `--right-arm-wheel-usb` can be used by itself; head and left-arm tools will simply report that their bus is not configured.
- The camera tool is designed to give the MCP client direct visual context. Pair it with the movement tools for a closed observe-act loop.
- Saved arm poses are restricted to simple file names to avoid path traversal from MCP callers.
