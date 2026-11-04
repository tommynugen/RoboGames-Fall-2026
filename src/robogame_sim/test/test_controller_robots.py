"""controller_robots' choices that need no ROS graph: which program runs a
robot's controls, and which topics are real robots."""

import pytest

pytest.importorskip("rclpy")

from robogame_sim import controller_robots  # noqa: E402
from robogame_sim.controller_robots import (  # noqa: E402
    robots_on,
    teleop_command,
)


@pytest.fixture(autouse=True)
def no_install(monkeypatch):
    monkeypatch.setattr(controller_robots, "executable",
                        lambda package, name: f"{package}/{name}")


def test_classic_controls_run_controller_input_on_the_robots_topics():
    command = teleop_command("classic", 2, "AA:BB")
    assert command[0] == "arena_perception/controller_input"
    assert "__ns:=/robot_2" in command and "robot_id:=2" in command
    assert not any("controller_address" in part for part in command)


def test_racing_controls_find_the_controllers_motion_sensor():
    command = teleop_command("racing", 3, "AA:BB")
    assert command[0] == "robogame_sim/racing_teleop"
    assert "controller_address:=AA:BB" in command


def test_a_real_robot_is_a_cmd_vel_something_listens_on():
    topics = [("/robot_1/cmd_vel", 1), ("/robot_2/cmd_vel", 0),
              ("/robot_12/cmd_vel", 2), ("/robot_3/joy", 1),
              ("/model/robot_4/cmd_vel", 1)]
    assert robots_on(topics) == {1, 12}
