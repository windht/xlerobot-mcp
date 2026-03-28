from pathlib import Path

from xlerobot_mcp.controllers import MockServoController


def test_mock_motion_updates_pose(tmp_path: Path) -> None:
    controller = MockServoController(position_dir=tmp_path)

    controller.go_forward(1.0)
    controller.turn_left(90.0)
    controller.go_forward(1.0)

    state = controller.get_state_snapshot()
    pose = state["estimated_pose"]

    assert round(pose["x_m"], 3) == 1.0
    assert round(pose["y_m"], 3) == 1.0
    assert round(pose["yaw_deg"], 3) == 90.0
    assert state["last_action"]["action"] == "move_forward"


def test_mock_head_clamps_to_safe_range(tmp_path: Path) -> None:
    controller = MockServoController(position_dir=tmp_path)

    controller.turn_head_yaw(999.0)
    controller.turn_head_pitch(-10.0)

    state = controller.get_state_snapshot()

    assert state["head"]["yaw_deg"] == 120.0
    assert state["head"]["pitch_deg"] == 0.0


def test_saved_pose_round_trip(tmp_path: Path) -> None:
    controller = MockServoController(position_dir=tmp_path)

    controller.set_arm_position({"shoulder_pan": 12.0, "gripper": 3.5}, "right")
    path = controller.save_arm_position("demo_pose", "right")

    controller.set_arm_position({"shoulder_pan": 0.0, "gripper": 0.0}, "right")
    restored = controller.set_saved_position("demo_pose", "right")

    assert Path(path).exists()
    assert restored["shoulder_pan"] == 12.0
    assert restored["gripper"] == 3.5
