import math

import pytest

from robogame_sim.chase import (
    ARENA_HALF,
    ARM_REACH,
    STOW_ARM,
    WALL_MARGIN,
    Chaser,
    TargetTracker,
    arm_reach,
    drive_command,
    footprint_extent,
    standoff_distance,
)
from robogame_sim.firmware_model import ArmModel
from robogame_sim.terrain import HALF_LENGTH, HALF_WIDTH

ARGS = dict(limit=0.3, max_angular=1.5, linear_gain=1.2, angular_gain=3.0,
            turn_in_place=1.0)


def command(hx, hy, hyaw, wx, wy, remaining=None, **kwargs):
    if remaining is None:
        remaining = math.hypot(wx - hx, wy - hy)
    return drive_command(hx, hy, hyaw, wx, wy, remaining, **ARGS, **kwargs)


def test_drives_straight_at_a_waypoint_ahead():
    linear, angular, error = command(0, 0, 0, 0.4, 0)
    assert linear == 0.3 and angular == 0.0 and error == 0.0


def test_turns_in_place_when_the_waypoint_is_behind():
    linear, angular, _ = command(0, 0, 0, -0.4, 0.01)
    assert linear == 0.0 and angular == 1.5


def test_curves_toward_a_waypoint_to_the_side():
    linear, angular, _ = command(0, 0, 0, 0.2, 0.1)
    assert 0 < linear < 0.3 and angular > 0


def test_stops_where_the_route_runs_out():
    linear, _, _ = command(0, 0, 0, 0.4, 0, remaining=0.005)
    assert linear == 0.0


def test_lines_up_on_the_spot_before_a_gap_it_cannot_turn_in():
    # 0.3 rad off would be driven out of in a curve in the open.
    linear, angular, _ = command(0, 0, 0, 0.3, 0.1, line_up=True)
    assert linear == 0.0 and angular > 0
    assert command(0, 0, 0, 0.3, 0.1)[0] > 0


def test_backs_out_of_a_gap_it_cannot_turn_in():
    linear, angular, _ = command(0, 0, 0, -0.3, 0.0, can_turn=False)
    assert linear < 0 and angular == 0.0


def test_only_creeps_while_straightening_up_in_a_gap():
    linear, angular, _ = command(0, 0, 0, 0.3, 0.1, can_turn=False)
    assert 0 < linear <= 0.03 and angular > 0


def test_does_not_turn_standing_still_in_a_gap():
    _, angular, _ = command(0, 0, 0, 0.3, 0.1, remaining=0.0,
                            can_turn=False)
    assert angular == 0.0


def test_keeps_up_with_a_target_driving_away():
    linear, _, _ = command(0, 0, 0, 0.4, 0, remaining=0.03, keep_up=0.2)
    assert abs(linear - (1.2 * 0.03 + 0.2)) < 1e-9


def driving_tracker(x, y, yaw, speed, turn_rate, steps=20, dt=0.05):
    """A tracker fed a target driving a steady arc, ending at x, y, yaw."""
    points = []

    for _ in range(steps + 1):
        points.append((x, y, yaw))
        mid = yaw - turn_rate * dt / 2
        x -= speed * dt * math.cos(mid)
        y -= speed * dt * math.sin(mid)
        yaw -= turn_rate * dt

    tracker = TargetTracker(smoothing=1.0)

    for index, (px, py, pyaw) in enumerate(reversed(points)):
        tracker.update(px, py, index * dt, pyaw)

    return tracker


def test_tracker_measures_speed_and_turn_rate():
    tracker = driving_tracker(0.0, 0.0, 0.0, 0.2, 0.8)
    assert abs(tracker.speed - 0.2) < 1e-3
    assert abs(tracker.turn_rate - 0.8) < 1e-6
    assert abs(tracker.vx - 0.2) < 1e-3 and abs(tracker.vy) < 1e-3


def test_tracker_sees_reversing_as_negative_speed():
    tracker = driving_tracker(0.0, 0.0, 0.0, -0.15, 0.0)
    assert abs(tracker.speed + 0.15) < 1e-6


def test_prediction_follows_a_straight_line():
    tracker = driving_tracker(0.0, 0.0, 0.0, 0.1, 0.0)
    x, y = tracker.predict(1.0)
    assert abs(x - 0.1) < 1e-6 and abs(y) < 1e-6


def test_prediction_follows_an_arc():
    # Radius 0.25 anticlockwise: a quarter turn ends 0.25 ahead, 0.25 left.
    tracker = driving_tracker(0.0, 0.0, 0.0, 0.2, 0.8)
    x, y = tracker.predict(math.pi / 2 / 0.8)
    assert abs(x - 0.25) < 1e-3 and abs(y - 0.25) < 1e-3


def test_prediction_stays_off_the_edge():
    tracker = driving_tracker(0.4, 0.0, 0.0, 0.2, 0.0)
    x, _ = tracker.predict(2.0)
    assert abs(x - (ARENA_HALF - WALL_MARGIN)) < 1e-9


def chaser_after(tracker):
    chaser = Chaser()
    chaser.target = tracker
    return chaser


def test_still_target_is_aimed_at_where_it_is():
    chaser = chaser_after(driving_tracker(0.3, 0.1, 0.0, 0.0, 0.0))
    assert chaser.aim(0.0, 0.0, 0.32, 0.13) == (0.3, 0.1)


def test_moving_target_is_aimed_ahead_of_but_not_far_past():
    # Crossing 0.4 m ahead at 0.15 m/s: aim a little down its path.
    chaser = chaser_after(driving_tracker(0.4, 0.2, -math.pi / 2, 0.15, 0.0))
    x, y = chaser.aim(0.0, 0.0, math.hypot(0.4, 0.2), 0.13)
    assert abs(x - 0.4) < 1e-6
    assert 0.2 - 0.15 * chaser.lead_time - 1e-6 <= y < 0.2


def test_close_target_is_aimed_at_almost_where_it_is():
    chaser = chaser_after(driving_tracker(0.14, 0.0, math.pi / 2, 0.2, 0.0))
    _, y = chaser.aim(0.0, 0.0, 0.14, 0.13)
    assert y < 0.01


def test_stowing_puts_the_arm_over_the_chassis_from_any_angle():
    for shoulder, elbow in ((90, 90), (0, 0), (180, 0), (35, 160)):
        arm = ArmModel()
        arm.shoulder, arm.elbow = shoulder, elbow
        assert arm.apply("".join(STOW_ARM)) == ""
        assert (arm.shoulder, arm.elbow) == (180, 180)

    joints = arm.joint_positions()
    assert arm_reach(joints["shoulder"], joints["elbow"]) == HALF_LENGTH


def test_arm_out_reaches_to_its_fingertips():
    assert arm_reach(0.0, 0.0) == pytest.approx(ARM_REACH)


def test_footprint_reaches_to_fingertips_in_front():
    assert abs(footprint_extent(0.0) - ARM_REACH) < 1e-9
    assert abs(footprint_extent(math.pi) - HALF_LENGTH) < 1e-9
    assert abs(footprint_extent(math.pi / 2) - HALF_WIDTH) < 1e-9
    assert abs(footprint_extent(0.0, HALF_LENGTH) - HALF_LENGTH) < 1e-9


def test_standoff_allows_for_the_targets_arm_but_not_the_hunters():
    # Target at origin facing +x, hunter in front of it facing back.
    head_on = standoff_distance(0.5, 0.0, 0.0, 0.0, 0.0, clearance=0.03)
    from_behind = standoff_distance(-0.5, 0.0, 0.0, 0.0, 0.0, clearance=0.03)
    stowed = standoff_distance(0.5, 0.0, 0.0, 0.0, 0.0, clearance=0.03,
                               target_reach=HALF_LENGTH)
    assert head_on == pytest.approx(HALF_LENGTH + ARM_REACH + 0.03)
    assert from_behind == pytest.approx(2 * HALF_LENGTH + 0.03)
    assert stowed == pytest.approx(from_behind)


def test_holds_its_ground_when_the_target_comes_at_it():
    chaser = Chaser()
    chaser.track(0.15, 0.0, 0.0, math.pi)
    chaser.track(0.14, 0.0, 0.05, math.pi)
    linear, _ = chaser.update(0.05, 0.0, 0.0, 0.0)
    assert linear == 0.0 and chaser.gap < 0.0


def test_frees_itself_when_pushing_against_something():
    chaser = Chaser(stuck_time=1.0)
    chaser.track(0.6, 0.0, 0.0)
    now = 0.0
    commands = []

    # Driving at the target, but the hunter never moves.
    while now < 1.3:
        commands.append(chaser.update(now, 0.0, 0.0, 0.0))
        chaser.track(0.6, 0.0, now)
        now += 0.05

    assert commands[0][0] > 0
    assert chaser.status == "freeing itself" and commands[-1][0] < 0
