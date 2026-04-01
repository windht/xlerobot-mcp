"""Servo controllers adapted from RoboCrew's XLeRobot servo_controls.py."""

from __future__ import annotations

import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Literal, Mapping, Optional

ArmSide = Literal["left", "right", "both"]
TorqueTarget = Literal["all", "wheels", "arms", "head"]

DEFAULT_SPEED = 10_000
LINEAR_MPS = 0.25
ANGULAR_DPS = 100.0

ACTION_MAP = {
    "forward": {7: 1.0, 8: 0.0, 9: -1.0},
    "backward": {7: -1.0, 8: 0.0, 9: 1.0},
    "strafe_left": {7: -0.15, 8: 1.0, 9: -0.15},
    "strafe_right": {7: 0.15, 8: -1.0, 9: 0.15},
    "turn_left": {7: 1.0, 8: 1.0, 9: 1.0},
    "turn_right": {7: -1.0, 8: -1.0, 9: -1.0},
}

HEAD_SERVO_MAP = {"yaw": 7, "pitch": 8}
HEAD_YAW_LIMIT_DEG = (-120.0, 120.0)
HEAD_PITCH_LIMIT_DEG = (0.0, 85.0)

DEFAULT_ARM_SERVO_MAP = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

DEFAULT_ARM_POSITION_DIR = "~/.cache/xlerobot-mcp/positions"
DEFAULT_ARM_CALIBRATION_DIR = "~/.cache/robocrew/calibrations/"

POSE_NAME_PATTERN = re.compile(r"^[A-Za-z0-9._-]+$")


class ControllerError(RuntimeError):
    """Base controller error."""


class ConfigurationError(ControllerError):
    """Raised when the controller is not configured correctly."""


class HardwareDependencyError(ControllerError):
    """Raised when hardware-only dependencies are missing."""


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _timestamp() -> str:
    return _utc_now().isoformat().replace("+00:00", "Z")


def _clamp(value: float, bounds: tuple[float, float]) -> float:
    low, high = bounds
    return max(low, min(high, float(value)))


def _normalize_angle(degrees: float) -> float:
    normalized = (float(degrees) + 180.0) % 360.0 - 180.0
    return 180.0 if normalized == -180.0 else normalized


def _validate_pose_name(position_name: str) -> str:
    base_name = position_name.removesuffix(".json")
    if not base_name or not POSE_NAME_PATTERN.fullmatch(base_name):
        raise ValueError(
            "position_name must contain only letters, numbers, '.', '_' or '-'."
        )
    return base_name


def _pick_non_degree_norm_mode() -> Any:
    lerobot = _import_lerobot()
    MotorNormMode = lerobot["MotorNormMode"]
    for name in ("RANGE_0_4095", "RANGE_0_100", "RANGE_M100_100"):
        mode = getattr(MotorNormMode, name, None)
        if mode is not None:
            return mode
    for mode in MotorNormMode:
        if mode != MotorNormMode.DEGREES:
            return mode
    return MotorNormMode.DEGREES


def _load_arm_servo_map(file_name: str) -> Dict[str, int]:
    local_path = Path(__file__).resolve().with_name(file_name)
    repo_path = Path(__file__).resolve().parents[2] / file_name
    if local_path.exists() or repo_path.exists():
        source_path = local_path if local_path.exists() else repo_path
        config = json.loads(source_path.read_text(encoding="utf-8"))
        return {name: int(item["id"]) for name, item in config.items()}
    return DEFAULT_ARM_SERVO_MAP.copy()


ARM_SERVO_MAPS = {
    "left": _load_arm_servo_map("left_arm.json"),
    "right": _load_arm_servo_map("right_arm.json"),
}


def _default_calibration(ids: tuple[int, ...]) -> Dict[int, Any]:
    lerobot = _import_lerobot()
    MotorCalibration = lerobot["MotorCalibration"]
    return {
        sid: MotorCalibration(
            id=sid,
            drive_mode=0,
            homing_offset=0,
            range_min=0,
            range_max=4095,
        )
        for sid in ids
    }


def _run_lerobot_calibrate(port: str, calibration_id: str, output_path: Path) -> None:
    env = dict(os.environ)
    env["HF_LEROBOT_CALIBRATION"] = str(Path(DEFAULT_ARM_CALIBRATION_DIR).expanduser())
    cmd = [
        sys.executable,
        "-m",
        "lerobot.scripts.lerobot_calibrate",
        "--robot.type=so101_follower",
        f"--robot.port={port}",
        f"--robot.id={calibration_id}",
    ]
    subprocess.run(cmd, check=True, env=env)
    generated = (
        Path(env["HF_LEROBOT_CALIBRATION"]).expanduser()
        / "robots"
        / "so_follower"
        / f"{calibration_id}.json"
    )
    if generated.exists():
        output_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(generated, output_path)


def _load_arm_calibration(
    file_name: str,
    ids: tuple[int, ...],
    arm_usb_port: Optional[str] = None,
) -> Dict[int, Any]:
    lerobot = _import_lerobot()
    MotorCalibration = lerobot["MotorCalibration"]

    path = Path(DEFAULT_ARM_CALIBRATION_DIR).expanduser() / file_name
    if not path.exists():
        if arm_usb_port:
            calibration_id = Path(file_name).stem
            try:
                _run_lerobot_calibrate(arm_usb_port, calibration_id, path)
            except Exception as exc:  # pragma: no cover - depends on live hardware
                print(
                    f"Warning: auto calibration failed for '{calibration_id}': {exc}",
                    file=sys.stderr,
                )
        if not path.exists():
            print(
                f"Warning: calibration file missing: '{path}'. Using default calibration.",
                file=sys.stderr,
            )
        return _default_calibration(ids)

    data = json.loads(path.read_text(encoding="utf-8"))
    loaded = {
        int(item["id"]): MotorCalibration(
            id=int(item["id"]),
            drive_mode=int(item.get("drive_mode", 0)),
            homing_offset=int(item.get("homing_offset", 0)),
            range_min=int(item.get("range_min", 0)),
            range_max=int(item.get("range_max", 4095)),
        )
        for item in data.values()
        if isinstance(item, dict) and "id" in item
    }
    for sid, cal in _default_calibration(ids).items():
        loaded.setdefault(sid, cal)
    return loaded


def _import_lerobot() -> Dict[str, Any]:
    try:
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
    except ImportError as exc:  # pragma: no cover - exercised only without dependency
        raise HardwareDependencyError(
            "The 'lerobot' package is required for --backend hardware. "
            "Install it with `uv sync --extra hardware`, run via "
            "`uvx --from \"xlerobot-mcp[hardware]\" --torch-backend cpu xlerobot-mcp --backend hardware ...`, "
            "or install manually with `pip install 'lerobot[feetech]'`."
        ) from exc

    try:
        import scservo_sdk  # noqa: F401
    except ImportError as exc:  # pragma: no cover - exercised only without dependency
        raise HardwareDependencyError(
            "The Feetech servo SDK is missing: Python module 'scservo_sdk' not found. "
            "Re-run `uv sync --extra hardware` (or use "
            "`uvx --from \"xlerobot-mcp[hardware]\" --torch-backend cpu ...`) "
            "so the `lerobot[feetech]` dependency is installed. "
            "If you manage packages manually, install `feetech-servo-sdk`."
        ) from exc

    return {
        "Motor": Motor,
        "MotorCalibration": MotorCalibration,
        "MotorNormMode": MotorNormMode,
        "FeetechMotorsBus": FeetechMotorsBus,
        "OperatingMode": OperatingMode,
    }


class _TrackedStateMixin:
    def _init_tracking(
        self,
        *,
        backend: str,
        right_arm_wheel_usb: Optional[str],
        left_arm_head_usb: Optional[str],
        speed: int,
        position_dir: str | Path,
    ) -> None:
        self.backend = backend
        self.right_arm_wheel_usb = right_arm_wheel_usb
        self.left_arm_head_usb = left_arm_head_usb
        self.speed = int(speed)
        self.position_dir = str(position_dir)
        self.action_map = ACTION_MAP
        self._wheel_ids = tuple(next(iter(self.action_map.values())).keys())
        self._head_ids = tuple(HEAD_SERVO_MAP.values())
        self._right_arm_ids = tuple(ARM_SERVO_MAPS["right"].values())
        self._left_arm_ids = tuple(ARM_SERVO_MAPS["left"].values())
        self._head_positions = {HEAD_SERVO_MAP["yaw"]: 0.0, HEAD_SERVO_MAP["pitch"]: 0.0}
        self._arm_positions_right = {
            name: 0.0 for name in ARM_SERVO_MAPS["right"].keys()
        }
        self._arm_positions_left = {
            name: 0.0 for name in ARM_SERVO_MAPS["left"].keys()
        }
        self._arm_positions = {name: 0.0 for name in ARM_SERVO_MAPS["right"].keys()}
        self._torque_enabled = {"wheels": True, "arms": True, "head": True}
        self._estimated_pose = {"x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0}
        self._connected = True
        self._last_action_report: Optional[Dict[str, Any]] = None
        self._action_counter = 0

    def _refresh_torque_state_from_capabilities(self) -> None:
        capabilities = self._capabilities()
        self._torque_enabled["wheels"] = capabilities["wheels"]
        self._torque_enabled["head"] = capabilities["head"]
        self._torque_enabled["arms"] = capabilities["left_arm"] or capabilities["right_arm"]

    def _capabilities(self) -> Dict[str, bool]:
        if self.backend == "mock":
            return {
                "wheels": True,
                "head": True,
                "right_arm": True,
                "left_arm": True,
            }
        return {
            "wheels": hasattr(self, "wheel_bus"),
            "head": hasattr(self, "head_bus"),
            "right_arm": hasattr(self, "wheel_bus"),
            "left_arm": hasattr(self, "head_bus"),
        }

    def _next_action_id(self) -> str:
        self._action_counter += 1
        return f"act_{self._action_counter:04d}"

    def _record_action(
        self,
        action: str,
        *,
        started_at: Optional[str] = None,
        finished_at: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        report = {
            "action_id": self._next_action_id(),
            "action": action,
            "status": "completed",
            "started_at": started_at or _timestamp(),
            "finished_at": finished_at or _timestamp(),
            "details": details or {},
            "estimated_pose": dict(self._estimated_pose),
        }
        self._last_action_report = report
        return report

    def _record_motion(
        self,
        action: str,
        *,
        forward_m: float = 0.0,
        lateral_left_m: float = 0.0,
        yaw_delta_deg: float = 0.0,
        duration_s: float,
        details: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        started_at = _timestamp()
        yaw_rad = math.radians(self._estimated_pose["yaw_deg"])
        self._estimated_pose["x_m"] += (
            forward_m * math.cos(yaw_rad) - lateral_left_m * math.sin(yaw_rad)
        )
        self._estimated_pose["y_m"] += (
            forward_m * math.sin(yaw_rad) + lateral_left_m * math.cos(yaw_rad)
        )
        self._estimated_pose["yaw_deg"] = _normalize_angle(
            self._estimated_pose["yaw_deg"] + yaw_delta_deg
        )
        finished_at = _timestamp()
        payload = {"duration_s": round(duration_s, 3)}
        if details:
            payload.update(details)
        return self._record_action(
            action,
            started_at=started_at,
            finished_at=finished_at,
            details=payload,
        )

    def _record_head_action(
        self,
        action: str,
        *,
        degrees: float,
        servo_name: str,
        clamped_to: float,
    ) -> Dict[str, Any]:
        return self._record_action(
            action,
            details={
                "requested_degrees": float(degrees),
                "servo": servo_name,
                "applied_degrees": float(clamped_to),
            },
        )

    def _record_arm_action(
        self,
        action: str,
        *,
        arm_side: ArmSide,
        positions: Mapping[str, float],
    ) -> Dict[str, Any]:
        return self._record_action(
            action,
            details={
                "arm_side": arm_side,
                "positions": {key: float(value) for key, value in positions.items()},
            },
        )

    def _record_torque_action(
        self,
        action: str,
        *,
        target: TorqueTarget,
        enabled: bool,
    ) -> Dict[str, Any]:
        return self._record_action(
            action,
            details={
                "target": target,
                "enabled": enabled,
                "torque_state": dict(self._torque_enabled),
            },
        )

    def _arm_position_file(self, position_name: str) -> Path:
        file_name = f"{_validate_pose_name(position_name)}.json"
        return Path(self.position_dir).expanduser() / file_name

    def get_last_action_report(self) -> Optional[Dict[str, Any]]:
        return dict(self._last_action_report) if self._last_action_report else None

    def get_state_snapshot(self) -> Dict[str, Any]:
        return {
            "backend": self.backend,
            "connected": self._connected,
            "ports": {
                "right_arm_wheel_usb": self.right_arm_wheel_usb,
                "left_arm_head_usb": self.left_arm_head_usb,
            },
            "capabilities": self._capabilities(),
            "speed": self.speed,
            "estimated_pose": {
                **self._estimated_pose,
                "source": "command_estimate",
            },
            "head": {
                "yaw_deg": self._head_positions[HEAD_SERVO_MAP["yaw"]],
                "pitch_deg": self._head_positions[HEAD_SERVO_MAP["pitch"]],
                "limits_deg": {
                    "yaw": HEAD_YAW_LIMIT_DEG,
                    "pitch": HEAD_PITCH_LIMIT_DEG,
                },
            },
            "arms": {
                "left": dict(self._arm_positions_left),
                "right": dict(self._arm_positions_right),
                "combined": dict(self._arm_positions),
            },
            "torque_enabled": dict(self._torque_enabled),
            "last_action": self.get_last_action_report(),
            "saved_pose_dir": str(Path(self.position_dir).expanduser()),
            "timestamp": _timestamp(),
        }

    def save_arm_position(
        self,
        position_name: str = "arm_positions",
        arm_side: ArmSide = "both",
    ) -> str:
        if arm_side == "left":
            positions = self._arm_positions_left.copy()
        elif arm_side == "right":
            positions = self._arm_positions_right.copy()
        else:
            positions = {
                "left": self._arm_positions_left.copy(),
                "right": self._arm_positions_right.copy(),
            }
        file_path = self._arm_position_file(position_name)
        file_path.parent.mkdir(parents=True, exist_ok=True)
        file_path.write_text(
            json.dumps({"arm_side": arm_side, "positions": positions}, indent=2),
            encoding="utf-8",
        )
        self._record_action(
            "save_arm_position",
            details={"arm_side": arm_side, "path": str(file_path)},
        )
        return str(file_path)

    def set_saved_position(
        self,
        position_name: str,
        arm_side: ArmSide = "both",
    ) -> Dict[str, float]:
        try:
            raw_data = json.loads(
                self._arm_position_file(position_name).read_text(encoding="utf-8")
            )
        except FileNotFoundError as exc:
            raise ControllerError(f"Saved pose '{position_name}' does not exist.") from exc

        if isinstance(raw_data, dict) and "arm_side" in raw_data and "positions" in raw_data:
            saved_side = raw_data["arm_side"]
            data = raw_data["positions"]
        elif isinstance(raw_data, dict) and "left" in raw_data and "right" in raw_data:
            saved_side = "both"
            data = raw_data
        else:
            raise ControllerError(
                "Position file is missing arm-side metadata. Re-save the pose with a recent format."
            )

        if saved_side != arm_side:
            raise ControllerError(
                f"Saved pose is for '{saved_side}' but requested '{arm_side}'."
            )

        if arm_side == "both":
            if isinstance(data, dict) and "left" in data and "right" in data:
                self.set_arm_position(data["left"], "left")
                self.set_arm_position(data["right"], "right")
                applied = self._arm_positions.copy()
            else:
                applied = self.set_arm_position(data, "both")
        elif isinstance(data, dict) and "left" in data and "right" in data:
            applied = self.set_arm_position(data[arm_side], arm_side)
        else:
            applied = self.set_arm_position(data, arm_side)

        self._record_action(
            "set_saved_position",
            details={
                "arm_side": arm_side,
                "position_name": _validate_pose_name(position_name),
                "positions": dict(applied),
            },
        )
        return applied


class MockServoController(_TrackedStateMixin):
    """In-memory controller for development and testing."""

    def __init__(
        self,
        *,
        right_arm_wheel_usb: Optional[str] = None,
        left_arm_head_usb: Optional[str] = None,
        speed: int = DEFAULT_SPEED,
        position_dir: str | Path = DEFAULT_ARM_POSITION_DIR,
    ) -> None:
        self._init_tracking(
            backend="mock",
            right_arm_wheel_usb=right_arm_wheel_usb,
            left_arm_head_usb=left_arm_head_usb,
            speed=speed,
            position_dir=position_dir,
        )

    def stop_wheels(self) -> Dict[str, Any]:
        return self._record_action("stop_wheels")

    def go_forward(self, meters: float) -> None:
        distance = float(meters)
        self._record_motion(
            "move_forward",
            forward_m=distance,
            duration_s=distance / LINEAR_MPS,
            details={"meters": distance},
        )

    def go_backward(self, meters: float) -> None:
        distance = float(meters)
        self._record_motion(
            "move_backward",
            forward_m=-distance,
            duration_s=distance / LINEAR_MPS,
            details={"meters": distance},
        )

    def turn_left(self, degrees: float) -> None:
        rotation = float(degrees)
        self._record_motion(
            "turn_left",
            yaw_delta_deg=rotation,
            duration_s=rotation / ANGULAR_DPS,
            details={"degrees": rotation},
        )

    def turn_right(self, degrees: float) -> None:
        rotation = float(degrees)
        self._record_motion(
            "turn_right",
            yaw_delta_deg=-rotation,
            duration_s=rotation / ANGULAR_DPS,
            details={"degrees": rotation},
        )

    def strafe_left(self, meters: float) -> None:
        distance = float(meters)
        self._record_motion(
            "strafe_left",
            lateral_left_m=distance,
            duration_s=distance / LINEAR_MPS,
            details={"meters": distance},
        )

    def strafe_right(self, meters: float) -> None:
        distance = float(meters)
        self._record_motion(
            "strafe_right",
            lateral_left_m=-distance,
            duration_s=distance / LINEAR_MPS,
            details={"meters": distance},
        )

    def turn_head_to_vla_position(self, pitch_deg: float = 45.0) -> None:
        self.turn_head_pitch(pitch_deg)
        self.turn_head_yaw(0.0)
        self._record_action(
            "turn_head_to_vla_position",
            details={"pitch_deg": float(_clamp(pitch_deg, HEAD_PITCH_LIMIT_DEG))},
        )

    def reset_head_position(self) -> None:
        self.turn_head_pitch(22.0)
        self.turn_head_yaw(0.0)
        self._record_action("reset_head_position")

    def enable_torque(self, target: TorqueTarget = "all") -> None:
        self._set_torque(True, target)

    def disable_torque(self, target: TorqueTarget = "all") -> None:
        self._set_torque(False, target)

    def _set_torque(self, enabled: bool, target: TorqueTarget) -> None:
        selected = ("wheels", "head", "arms") if target == "all" else (target,)
        for key in selected:
            self._torque_enabled[key] = enabled
        self._record_torque_action(
            "enable_torque" if enabled else "disable_torque",
            target=target,
            enabled=enabled,
        )

    def turn_head_yaw(self, degrees: float) -> Dict[int, float]:
        applied = _clamp(degrees, HEAD_YAW_LIMIT_DEG)
        payload = {HEAD_SERVO_MAP["yaw"]: applied}
        self._head_positions.update(payload)
        self._record_head_action(
            "turn_head_yaw",
            degrees=degrees,
            servo_name="yaw",
            clamped_to=applied,
        )
        return payload

    def turn_head_pitch(self, degrees: float) -> Dict[int, float]:
        applied = _clamp(degrees, HEAD_PITCH_LIMIT_DEG)
        payload = {HEAD_SERVO_MAP["pitch"]: applied}
        self._head_positions.update(payload)
        self._record_head_action(
            "turn_head_pitch",
            degrees=degrees,
            servo_name="pitch",
            clamped_to=applied,
        )
        return payload

    def set_arm_position(
        self,
        positions: Mapping[str, float],
        arm_side: ArmSide = "both",
    ) -> Dict[str, float]:
        payload = {name: float(value) for name, value in positions.items()}
        if arm_side in ("right", "both"):
            self._arm_positions_right.update(
                {name: value for name, value in payload.items() if name in ARM_SERVO_MAPS["right"]}
            )
        if arm_side in ("left", "both"):
            self._arm_positions_left.update(
                {name: value for name, value in payload.items() if name in ARM_SERVO_MAPS["left"]}
            )
        self._arm_positions.update(payload)
        self._record_arm_action("set_arm_position", arm_side=arm_side, positions=payload)
        if arm_side == "left":
            return self._arm_positions_left.copy()
        if arm_side == "right":
            return self._arm_positions_right.copy()
        return self._arm_positions.copy()

    def read_arm_present_position(self, arm_side: ArmSide = "both") -> Dict[str, float]:
        if arm_side == "left":
            return self._arm_positions_left.copy()
        if arm_side == "right":
            return self._arm_positions_right.copy()
        return self._arm_positions.copy()

    def disconnect(self) -> None:
        self._connected = False
        self._record_action("disconnect")


class HardwareServoController(_TrackedStateMixin):
    """Servo controller backed by the live LeRobot motor buses."""

    def __init__(
        self,
        right_arm_wheel_usb: Optional[str] = None,
        left_arm_head_usb: Optional[str] = None,
        *,
        speed: int = DEFAULT_SPEED,
        position_dir: str | Path = DEFAULT_ARM_POSITION_DIR,
        action_map: Optional[Mapping[str, Mapping[int, float]]] = None,
    ) -> None:
        if not right_arm_wheel_usb and not left_arm_head_usb:
            raise ConfigurationError(
                "Hardware backend needs at least one USB port. "
                "Pass --right-arm-wheel-usb and/or --left-arm-head-usb."
            )

        self._init_tracking(
            backend="hardware",
            right_arm_wheel_usb=right_arm_wheel_usb,
            left_arm_head_usb=left_arm_head_usb,
            speed=speed,
            position_dir=position_dir,
        )
        self.action_map = ACTION_MAP if action_map is None else dict(action_map)
        self._wheel_ids = tuple(next(iter(self.action_map.values())).keys())
        lerobot = _import_lerobot()
        Motor = lerobot["Motor"]
        MotorNormMode = lerobot["MotorNormMode"]
        FeetechMotorsBus = lerobot["FeetechMotorsBus"]

        self._position_norm_mode = _pick_non_degree_norm_mode()
        self._head_norm_mode = MotorNormMode.DEGREES
        self._right_arm_ids = tuple(ARM_SERVO_MAPS["right"].values())
        self._left_arm_ids = tuple(ARM_SERVO_MAPS["left"].values())

        if right_arm_wheel_usb:
            right_arm_calibration = _load_arm_calibration(
                "right_arm.json",
                self._right_arm_ids,
                right_arm_wheel_usb,
            )
            arm_motors = {
                aid: Motor(aid, "sts3215", self._position_norm_mode)
                for aid in self._right_arm_ids
            }
            self.wheel_bus = FeetechMotorsBus(
                port=right_arm_wheel_usb,
                motors={
                    **arm_motors,
                    7: Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                    8: Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                    9: Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
                },
                calibration=right_arm_calibration,
            )
            self.wheel_bus.connect()
            self.apply_wheel_modes()
            self.apply_arm_modes()

        if left_arm_head_usb:
            left_arm_calibration = _load_arm_calibration(
                "left_arm.json",
                self._left_arm_ids,
                left_arm_head_usb,
            )
            head_calibration = {
                **left_arm_calibration,
                7: lerobot["MotorCalibration"](
                    id=7,
                    drive_mode=0,
                    homing_offset=0,
                    range_min=0,
                    range_max=4095,
                ),
                8: lerobot["MotorCalibration"](
                    id=8,
                    drive_mode=0,
                    homing_offset=0,
                    range_min=0,
                    range_max=4095,
                ),
            }
            left_arm_motors = {
                aid: Motor(aid, "sts3215", self._position_norm_mode)
                for aid in self._left_arm_ids
            }
            self.head_bus = FeetechMotorsBus(
                port=left_arm_head_usb,
                motors={
                    **left_arm_motors,
                    HEAD_SERVO_MAP["yaw"]: Motor(
                        HEAD_SERVO_MAP["yaw"], "sts3215", self._head_norm_mode
                    ),
                    HEAD_SERVO_MAP["pitch"]: Motor(
                        HEAD_SERVO_MAP["pitch"], "sts3215", self._head_norm_mode
                    ),
                },
                calibration=head_calibration,
            )
            self.head_bus.connect()
            self.apply_head_modes()
            self.apply_arm_modes()

        self._refresh_torque_state_from_capabilities()

    def _require_bus(self, attribute: str, description: str) -> Any:
        if not hasattr(self, attribute):
            raise ControllerError(
                f"{description} is unavailable because the matching USB bus is not configured."
            )
        return getattr(self, attribute)

    def _wheels_stop(self) -> None:
        bus = self._require_bus("wheel_bus", "Wheel control")
        payload = {wid: 0 for wid in self._wheel_ids}
        bus.sync_write("Goal_Velocity", payload)

    def stop_wheels(self) -> Dict[str, Any]:
        self._wheels_stop()
        return self._record_action("stop_wheels")

    def _wheels_run(self, action: str, duration: float) -> None:
        if duration <= 0:
            return
        bus = self._require_bus("wheel_bus", "Wheel control")
        multipliers = self.action_map[action]
        payload = {wid: int(self.speed * factor) for wid, factor in multipliers.items()}
        bus.sync_write("Goal_Velocity", payload)
        time.sleep(duration)
        self._wheels_stop()

    def go_forward(self, meters: float) -> None:
        distance = float(meters)
        self._wheels_run("forward", distance / LINEAR_MPS)
        self._record_motion(
            "move_forward",
            forward_m=distance,
            duration_s=distance / LINEAR_MPS,
            details={"meters": distance},
        )

    def go_backward(self, meters: float) -> None:
        distance = float(meters)
        self._wheels_run("backward", distance / LINEAR_MPS)
        self._record_motion(
            "move_backward",
            forward_m=-distance,
            duration_s=distance / LINEAR_MPS,
            details={"meters": distance},
        )

    def turn_left(self, degrees: float) -> None:
        rotation = float(degrees)
        self._wheels_run("turn_left", rotation / ANGULAR_DPS)
        self._record_motion(
            "turn_left",
            yaw_delta_deg=rotation,
            duration_s=rotation / ANGULAR_DPS,
            details={"degrees": rotation},
        )

    def turn_right(self, degrees: float) -> None:
        rotation = float(degrees)
        self._wheels_run("turn_right", rotation / ANGULAR_DPS)
        self._record_motion(
            "turn_right",
            yaw_delta_deg=-rotation,
            duration_s=rotation / ANGULAR_DPS,
            details={"degrees": rotation},
        )

    def strafe_left(self, meters: float) -> None:
        distance = float(meters)
        self._wheels_run("strafe_left", distance / LINEAR_MPS)
        self._record_motion(
            "strafe_left",
            lateral_left_m=distance,
            duration_s=distance / LINEAR_MPS,
            details={"meters": distance},
        )

    def strafe_right(self, meters: float) -> None:
        distance = float(meters)
        self._wheels_run("strafe_right", distance / LINEAR_MPS)
        self._record_motion(
            "strafe_right",
            lateral_left_m=-distance,
            duration_s=distance / LINEAR_MPS,
            details={"meters": distance},
        )

    def turn_head_to_vla_position(self, pitch_deg: float = 45.0) -> None:
        self.turn_head_pitch(pitch_deg)
        self.turn_head_yaw(0.0)
        time.sleep(0.9)
        self._record_action(
            "turn_head_to_vla_position",
            details={"pitch_deg": float(_clamp(pitch_deg, HEAD_PITCH_LIMIT_DEG))},
        )

    def reset_head_position(self) -> None:
        self.turn_head_pitch(22.0)
        self.turn_head_yaw(0.0)
        time.sleep(0.9)
        self._record_action("reset_head_position")

    def apply_wheel_modes(self) -> None:
        OperatingMode = _import_lerobot()["OperatingMode"]
        bus = self._require_bus("wheel_bus", "Wheel control")
        for wid in self._wheel_ids:
            bus.write("Operating_Mode", wid, OperatingMode.VELOCITY.value)
        bus.enable_torque()

    def _set_position_mode(self, bus: Any, ids: tuple[int, ...]) -> None:
        OperatingMode = _import_lerobot()["OperatingMode"]
        for sid in ids:
            bus.write("Operating_Mode", sid, OperatingMode.POSITION.value)
        bus.enable_torque()

    def apply_arm_modes(self) -> None:
        if hasattr(self, "wheel_bus"):
            self._set_position_mode(self.wheel_bus, self._right_arm_ids)
        if hasattr(self, "head_bus"):
            self._set_position_mode(self.head_bus, self._left_arm_ids)

    def apply_head_modes(self) -> None:
        bus = self._require_bus("head_bus", "Head control")
        self._set_position_mode(bus, self._head_ids)

    def _set_bus_torque(self, bus: Any, ids: tuple[int, ...], enabled: bool) -> None:
        fn = getattr(bus, "enable_torque" if enabled else "disable_torque", None)
        if fn:
            fn()
            return
        for sid in ids:
            bus.write("Torque_Enable", sid, int(enabled))

    def _set_torque(self, enabled: bool, target: TorqueTarget = "all") -> None:
        groups = {
            "wheels": ((getattr(self, "wheel_bus", None), self._wheel_ids),),
            "head": ((getattr(self, "head_bus", None), self._head_ids),),
            "arms": (
                (getattr(self, "wheel_bus", None), self._right_arm_ids),
                (getattr(self, "head_bus", None), self._left_arm_ids),
            ),
        }
        selected = ("wheels", "head", "arms") if target == "all" else (target,)
        for key in selected:
            for bus, ids in groups[key]:
                if bus:
                    self._set_bus_torque(bus, ids, enabled)
            self._torque_enabled[key] = enabled
        self._record_torque_action(
            "enable_torque" if enabled else "disable_torque",
            target=target,
            enabled=enabled,
        )

    def enable_torque(self, target: TorqueTarget = "all") -> None:
        self._set_torque(True, target)

    def disable_torque(self, target: TorqueTarget = "all") -> None:
        self._set_torque(False, target)

    def turn_head_yaw(self, degrees: float) -> Dict[int, float]:
        bus = self._require_bus("head_bus", "Head control")
        applied = _clamp(degrees, HEAD_YAW_LIMIT_DEG)
        payload = {HEAD_SERVO_MAP["yaw"]: applied}
        bus.sync_write("Goal_Position", payload)
        self._head_positions.update(payload)
        self._record_head_action(
            "turn_head_yaw",
            degrees=degrees,
            servo_name="yaw",
            clamped_to=applied,
        )
        return payload

    def turn_head_pitch(self, degrees: float) -> Dict[int, float]:
        bus = self._require_bus("head_bus", "Head control")
        applied = _clamp(degrees, HEAD_PITCH_LIMIT_DEG)
        payload = {HEAD_SERVO_MAP["pitch"]: applied}
        bus.sync_write("Goal_Position", payload)
        self._head_positions.update(payload)
        self._record_head_action(
            "turn_head_pitch",
            degrees=degrees,
            servo_name="pitch",
            clamped_to=applied,
        )
        return payload

    def set_arm_position(
        self,
        positions: Mapping[str, float],
        arm_side: ArmSide = "both",
    ) -> Dict[str, float]:
        right_map = ARM_SERVO_MAPS["right"]
        left_map = ARM_SERVO_MAPS["left"]

        if arm_side in ("right", "both"):
            bus = self._require_bus("wheel_bus", "Right-arm control")
            right_payload = {
                right_map[name]: float(value)
                for name, value in positions.items()
                if name in right_map
            }
            if right_payload:
                bus.sync_write("Goal_Position", right_payload)
                self._arm_positions_right.update(
                    {
                        name: float(value)
                        for name, value in positions.items()
                        if name in right_map
                    }
                )

        if arm_side in ("left", "both"):
            bus = self._require_bus("head_bus", "Left-arm control")
            left_payload = {
                left_map[name]: float(value)
                for name, value in positions.items()
                if name in left_map
            }
            if left_payload:
                bus.sync_write("Goal_Position", left_payload)
                self._arm_positions_left.update(
                    {
                        name: float(value)
                        for name, value in positions.items()
                        if name in left_map
                    }
                )

        self._arm_positions.update(
            {name: float(value) for name, value in positions.items()}
        )
        self._record_arm_action("set_arm_position", arm_side=arm_side, positions=positions)
        if arm_side == "left":
            return self._arm_positions_left.copy()
        if arm_side == "right":
            return self._arm_positions_right.copy()
        return self._arm_positions.copy()

    def read_arm_present_position(self, arm_side: ArmSide = "both") -> Dict[str, float]:
        right_map = ARM_SERVO_MAPS["right"]
        left_map = ARM_SERVO_MAPS["left"]

        def _read_present(bus: Any, ids: tuple[int, ...]) -> Dict[int, float]:
            try:
                raw = bus.sync_read("Present_Position", list(ids))
                if isinstance(raw, dict):
                    return {int(k): float(v) for k, v in raw.items()}
                return {sid: float(value) for sid, value in zip(ids, raw)}
            except Exception:
                return {sid: float(bus.read("Present_Position", sid)) for sid in ids}

        if arm_side in ("right", "both") and hasattr(self, "wheel_bus"):
            right_data = _read_present(self.wheel_bus, self._right_arm_ids)
            self._arm_positions_right.update(
                {
                    name: float(right_data[sid])
                    for name, sid in right_map.items()
                    if sid in right_data
                }
            )

        if arm_side in ("left", "both") and hasattr(self, "head_bus"):
            left_data = _read_present(self.head_bus, self._left_arm_ids)
            self._arm_positions_left.update(
                {
                    name: float(left_data[sid])
                    for name, sid in left_map.items()
                    if sid in left_data
                }
            )

        self._arm_positions.update(self._arm_positions_left)
        self._arm_positions.update(self._arm_positions_right)
        if arm_side == "left":
            return self._arm_positions_left.copy()
        if arm_side == "right":
            return self._arm_positions_right.copy()
        return self._arm_positions.copy()

    def disconnect(self) -> None:
        if hasattr(self, "wheel_bus"):
            self._wheels_stop()
            time.sleep(0.5)
            self.wheel_bus.disconnect()
        if hasattr(self, "head_bus"):
            self.head_bus.disconnect()
        self._connected = False
        self._torque_enabled = {"wheels": False, "arms": False, "head": False}
        self._record_action("disconnect")
