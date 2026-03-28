from pathlib import Path

import xlerobot_mcp.controllers as controllers


class _FakeMotor:
    def __init__(self, *args, **kwargs) -> None:
        self.args = args
        self.kwargs = kwargs


class _FakeMotorCalibration:
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs


class _FakeModeValue:
    def __init__(self, value: int) -> None:
        self.value = value


class _FakeMotorNormMode:
    DEGREES = "degrees"
    RANGE_M100_100 = "range_m100_100"


class _FakeOperatingMode:
    VELOCITY = _FakeModeValue(1)
    POSITION = _FakeModeValue(2)


class _FakeBus:
    def __init__(self, port, motors, calibration) -> None:
        self.port = port
        self.motors = motors
        self.calibration = calibration
        self.connected = False

    def connect(self) -> None:
        self.connected = True

    def write(self, *args, **kwargs) -> None:
        return None

    def enable_torque(self) -> None:
        return None

    def disable_torque(self) -> None:
        return None

    def sync_write(self, *args, **kwargs) -> None:
        return None

    def disconnect(self) -> None:
        self.connected = False


def test_hardware_controller_allows_right_bus_only(monkeypatch, tmp_path: Path) -> None:
    calibration_calls = []

    def fake_import_lerobot():
        return {
            "Motor": _FakeMotor,
            "MotorCalibration": _FakeMotorCalibration,
            "MotorNormMode": _FakeMotorNormMode,
            "FeetechMotorsBus": _FakeBus,
            "OperatingMode": _FakeOperatingMode,
        }

    def fake_load_arm_calibration(file_name, ids, arm_usb_port=None):
        calibration_calls.append((file_name, tuple(ids), arm_usb_port))
        return {sid: _FakeMotorCalibration(id=sid) for sid in ids}

    monkeypatch.setattr(controllers, "_import_lerobot", fake_import_lerobot)
    monkeypatch.setattr(controllers, "_pick_non_degree_norm_mode", lambda: "position")
    monkeypatch.setattr(controllers, "_load_arm_calibration", fake_load_arm_calibration)

    controller = controllers.HardwareServoController(
        right_arm_wheel_usb="/dev/ttyUSB0",
        position_dir=tmp_path,
    )

    state = controller.get_state_snapshot()

    assert calibration_calls == [
        ("right_arm.json", controller._right_arm_ids, "/dev/ttyUSB0")
    ]
    assert state["capabilities"] == {
        "wheels": True,
        "head": False,
        "right_arm": True,
        "left_arm": False,
    }
    assert state["torque_enabled"] == {
        "wheels": True,
        "arms": True,
        "head": False,
    }
