"""The robot console's Autopilot, closing the loop in chase_sim's 2D
stand-in for the sim."""

import math
import os

import pytest

from robogame_sim import autopilot, chase_sim
from robogame_sim.autopilot import ARRIVE, Autopilot
from robogame_sim.chase import STOW_ARM
from robogame_sim.chase_sim import DT, TICK, Robot

DEFAULT_LAYOUT = os.path.join(os.path.dirname(__file__), "..", "layouts",
                              "default.json")


@pytest.fixture(scope="module")
def default():
    return chase_sim.load_world(DEFAULT_LAYOUT)


def drive(pilot, robot, others, seconds, until=None):
    """Runs the pilot on robot until its queue is empty, or until says so,
    and returns how long that took, or None if it never did. others maps
    robot numbers to Robots that stand still."""
    now, next_tick = 0.0, 0.0
    command = (0.0, 0.0)
    while now < seconds:
        if now >= next_tick:
            next_tick += TICK
            seen = {k: (o.x, o.y, o.yaw, now, None)
                    for k, o in others.items()}
            linear, angular, _ = pilot.update(now, robot.x, robot.y,
                                              robot.yaw, seen)
            command = (linear, angular)
            done = until() if until else (not pilot.queue
                                          and not pilot.arm_queue)
            if done:
                return now
        robot.step(*command)
        now += DT
    return None


def test_arm_is_stowed_before_it_drives():
    pilot = Autopilot(1)
    pilot.go(autopilot.point(0.3, 0.0))
    pilot.stow()

    sent = ""
    for _ in range(len(STOW_ARM)):
        linear, angular, arm = pilot.update(0.0, 0.0, 0.0, 0.0, {})
        assert (linear, angular) == (0.0, 0.0)
        sent += arm
    # With no joint states, as on a real robot, every step goes out.
    assert sent == STOW_ARM

    linear, _, arm = pilot.update(0.0, 0.0, 0.0, 0.0, {})
    assert arm is None and linear > 0.0


def test_stowing_stops_once_the_arm_is_up():
    pilot = Autopilot(1)
    pilot.stow()
    pilot.update(0.0, 0.0, 0.0, 0.0, {})
    pilot.set_arm(1.6, 1.6)
    pilot.update(0.0, 0.0, 0.0, 0.0, {})
    assert pilot.arm_queue == ""


def test_goes_up_the_ramp_to_a_spot_on_the_platform(default):
    robot = Robot(default, -0.45, -0.45, 0.0)
    pilot = Autopilot(1, default.terrain)
    pilot.go(autopilot.point(0.0, 0.05))

    assert drive(pilot, robot, {}, 30.0) is not None
    assert math.dist((robot.x, robot.y), (0.0, 0.05)) < ARRIVE + 0.01
    assert default.terrain.height_at(robot.x, robot.y) == pytest.approx(
        0.0254, abs=1e-3)
    assert pilot.last == "reached 0.00, 0.05"
    assert pilot.status == "idle"


def test_works_through_its_queue_in_order(default):
    robot = Robot(default, -0.45, -0.45, 0.0)
    still = Robot(default, -0.45, 0.45, 0.0)
    pilot = Autopilot(1, default.terrain)
    pilot.go(autopilot.point(0.45, -0.45))
    pilot.go(autopilot.robot(2), append=True)

    reached_spot = drive(pilot, robot, {2: still}, 30.0,
                         until=lambda: len(pilot.queue) == 1)
    assert reached_spot is not None
    assert math.dist((robot.x, robot.y), (0.45, -0.45)) < ARRIVE + 0.01

    assert drive(pilot, robot, {2: still}, 40.0) is not None
    assert pilot.last == "reached robot 2"
    # Beside it, not on it.
    gap = math.dist((robot.x, robot.y), (still.x, still.y))
    assert 0.08 < gap < 0.3


def test_a_spot_it_cannot_reach_ends_as_close_as_it_gets():
    # A 2 inch platform no ramp leads up to.
    world = chase_sim.load_world({"size": 12, "modules": [
        {"type": "platform_2", "col": c, "row": r}
        for c in (5, 6) for r in (5, 6)]})
    robot = Robot(world, -0.45, -0.45, 0.0)
    pilot = Autopilot(1, world.terrain)
    pilot.go(autopilot.point(0.0, 0.0))

    assert drive(pilot, robot, {}, 30.0) is not None
    assert pilot.last.startswith("could not reach")
    assert world.terrain.height_at(robot.x, robot.y) == 0.0
    assert math.dist((robot.x, robot.y), (0.0, 0.0)) < 0.25


def test_chase_mode_keeps_after_a_robot_that_moves(default):
    robot = Robot(default, -0.45, -0.45, 0.0)
    target = Robot(default, 0.45, -0.45, math.pi)
    pilot = Autopilot(1, default.terrain)
    pilot.set_mode(autopilot.CHASE)
    pilot.chase(2)

    def with_it():
        return math.dist((robot.x, robot.y), (target.x, target.y)) < 0.3

    assert drive(pilot, robot, {2: target}, 20.0, until=with_it) is not None
    # It does not count as done on reaching it, as going to a robot does.
    target.x, target.y = -0.45, 0.45
    assert drive(pilot, robot, {2: target}, 30.0, until=with_it) is not None
    assert pilot.status.startswith("chasing robot 2")


def test_waits_for_a_robot_it_cannot_see():
    pilot = Autopilot(1)
    pilot.go(autopilot.robot(3))
    assert pilot.update(0.0, 0.0, 0.0, 0.0, {})[:2] == (0.0, 0.0)
    assert pilot.status == "waiting to see robot 3"


def test_commands_are_checked():
    with pytest.raises(ValueError):
        autopilot.point("a", 0)
    with pytest.raises(ValueError):
        autopilot.point(math.nan, 0)
    with pytest.raises(ValueError):
        Autopilot(1).chase(1)
    with pytest.raises(ValueError):
        Autopilot(1).set_mode("dance")


def test_stop_clears_the_queue_and_the_chase():
    pilot = Autopilot(1)
    pilot.go(autopilot.point(0.1, 0.1))
    pilot.go(autopilot.point(0.2, 0.1), append=True)
    pilot.chase(2)
    pilot.remove(0)
    assert pilot.queue == [autopilot.point(0.2, 0.1)]
    pilot.stop()
    assert pilot.queue == [] and pilot.chase_target is None
