import math

from robogame_sim.chase import (
    ARM_REACH,
    TargetTracker,
    chase_command,
    footprint_extent,
    standoff_distance,
)

ARGS = dict(standoff=0.12, max_linear=0.3, max_angular=1.5,
            linear_gain=1.2, angular_gain=3.0, turn_in_place=1.0)


def command(hx, hy, hyaw, tx, ty):
    return chase_command(hx, hy, hyaw, tx, ty,
                         math.hypot(tx - hx, ty - hy), **ARGS)


def test_drives_straight_at_target_ahead():
    linear, angular, error = command(0, 0, 0, 0.4, 0)
    assert linear == 0.3 and angular == 0.0 and error == 0.0


def test_turns_in_place_when_target_is_behind():
    linear, angular, _ = command(0, 0, 0, -0.4, 0.01)
    assert linear == 0.0 and angular == 1.5


def test_curves_toward_target_to_the_side():
    linear, angular, _ = command(0, 0, 0, 0.2, 0.1)
    assert 0 < linear < 0.3 and angular > 0


def test_stops_at_standoff():
    linear, _, _ = command(0, 0, 0, 0.125, 0)
    assert linear == 0.0


def test_backs_off_when_too_close():
    linear, _, _ = command(0, 0, 0, 0.05, 0)
    assert -0.15 <= linear < 0


def test_tracker_estimates_velocity_and_leads():
    tracker = TargetTracker(smoothing=1.0)
    tracker.update(0.0, 0.0, 0.0)
    tracker.update(0.01, 0.0, 0.1)
    assert abs(tracker.vx - 0.1) < 1e-9

    x, y = tracker.predict(lead_time=0.5, max_lead=1.0)
    assert abs(x - 0.06) < 1e-9 and y == 0.0


def test_lead_is_capped_and_kept_off_walls():
    tracker = TargetTracker(smoothing=1.0)
    tracker.update(0.4, 0.0, 0.0)
    tracker.update(0.42, 0.0, 0.01)

    x, _ = tracker.predict(lead_time=1.0, max_lead=0.15)
    assert x == 0.44


def test_footprint_reaches_to_fingertips_in_front():
    assert abs(footprint_extent(0.0) - ARM_REACH) < 1e-9
    assert abs(footprint_extent(math.pi) - 0.04) < 1e-9
    assert abs(footprint_extent(math.pi / 2) - 0.035) < 1e-9


def test_standoff_allows_for_both_arms_head_on():
    # Target at origin facing +x, hunter in front of it facing back.
    head_on = standoff_distance(0.5, 0.0, 0.0, 0.0, 0.0, clearance=0.03)
    from_behind = standoff_distance(-0.5, 0.0, 0.0, 0.0, 0.0, clearance=0.03)
    assert abs(head_on - (2 * ARM_REACH + 0.03)) < 1e-9
    assert abs(from_behind - (ARM_REACH + 0.04 + 0.03)) < 1e-9
