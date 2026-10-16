import math

from robogame_sim.chase import (
    APPROACH_Y,
    ARM_REACH,
    CORNERS,
    FLOOR,
    LANE_HALF,
    PLATFORM,
    RAMP,
    RAMP_TOP_Y,
    TOP_Y,
    TargetTracker,
    chase_command,
    crosses_block,
    floor_path,
    follow_command,
    footprint_extent,
    in_block,
    level,
    route,
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


def test_prediction_stays_off_the_walls():
    tracker = driving_tracker(0.4, 0.0, 0.0, 0.2, 0.0)
    x, _ = tracker.predict(2.0)
    assert x == 0.44


def test_still_target_is_aimed_at_directly():
    tracker = TargetTracker()
    tracker.update(0.3, 0.1, 0.0)
    x, y, seconds = tracker.intercept(
        0.0, 0.0, 0.0, 0.1, 0.25, 1.5, 1.5)
    assert (x, y, seconds) == (0.3, 0.1, 0.0)


def test_crossing_target_is_met_ahead_of_where_it_is():
    # Target 0.4 m ahead, crossing left to right at 0.15 m/s.
    tracker = driving_tracker(0.4, 0.2, -math.pi / 2, 0.15, 0.0)
    x, y, seconds = tracker.intercept(
        0.0, 0.0, 0.0, 0.1, 0.25, 1.5, 1.5)
    assert 0.0 < seconds < 1.5
    assert abs(x - 0.4) < 1e-6 and y < 0.2
    assert abs(y - (0.2 - 0.15 * seconds)) < 1e-6


def test_target_out_of_reach_is_aimed_at_the_horizon():
    tracker = driving_tracker(0.0, 0.0, 0.0, 0.2, 0.0)
    _, _, seconds = tracker.intercept(
        -0.4, 0.0, 0.0, 0.1, 0.05, 1.5, 1.0)
    assert seconds == 1.0


def test_keeps_up_with_a_target_driving_away():
    # 3 cm past the standoff behind a target pulling away at 0.2 m/s.
    linear, _, _ = chase_command(
        0, 0, 0, 0.15, 0, 0.15, target_vx=0.2, **ARGS)
    assert abs(linear - (1.2 * 0.03 + 0.2)) < 1e-9


def test_speed_matching_fades_out_at_the_standoff():
    # Right at the standoff, so a target that stops is not run into.
    linear, _, _ = chase_command(
        0, 0, 0, 0.12, 0, 0.12, target_vx=0.2, **ARGS)
    assert linear == 0.0


def test_backs_off_from_the_target_even_when_aiming_past_it():
    linear, _, _ = chase_command(
        0, 0, 0, 0.4, 0, 0.05, target_vx=0.2, **ARGS)
    assert linear < 0


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


def drive_route(hx, hy, hyaw, tx, ty, steps=600, dt=0.05):
    """Drives a hunter after a still target the way chase_node does, and
    returns the hunter's centre at every step."""
    tracker = TargetTracker()
    tracker.update(tx, ty, 0.0)
    trail = [(hx, hy)]

    for _ in range(steps):
        standoff = standoff_distance(hx, hy, tx, ty, 0.0, 0.03)
        waypoint = route(hx, hy, tx, ty, tx, ty, standoff)

        if waypoint is None:
            linear, angular, _ = chase_command(
                hx, hy, hyaw, tx, ty, math.hypot(tx - hx, ty - hy),
                standoff, 0.3, 1.5, 1.2, 3.0, 1.0)
        else:
            linear, angular, _ = follow_command(
                hx, hy, hyaw, *waypoint, 0.3, 1.5, 1.2, 3.0, 1.0)

        hx += linear * dt * math.cos(hyaw)
        hy += linear * dt * math.sin(hyaw)
        hyaw += angular * dt
        trail.append((hx, hy))

    return trail


def assert_no_climbing_the_edge(trail):
    """The hunter only gets onto the raised block over the ramp's foot,
    with its wheels inside the ramp."""
    for (ax, ay), (bx, by) in zip(trail, trail[1:]):
        if level(ax, ay) == FLOOR and level(bx, by) != FLOOR:
            assert level(bx, by) == RAMP and abs(bx) <= LANE_HALF + 0.01
        if level(ax, ay) == FLOOR:
            assert not (in_block(bx, by) and abs(bx) > LANE_HALF + 0.01)


def test_level_of_each_part_of_the_arena():
    assert level(0.0, 0.0) == PLATFORM
    assert level(0.0, -0.25) == RAMP
    assert level(0.0, -0.35) == FLOOR
    assert level(0.3, 0.0) == FLOOR
    assert level(0.0, 0.25) == FLOOR


def test_block_check_sees_a_line_through_the_platform():
    assert crosses_block(-0.4, 0.0, 0.4, 0.0)
    assert not crosses_block(-0.4, 0.3, 0.4, 0.3)
    assert not crosses_block(-0.3, -0.4, -0.3, 0.4)


def test_floor_path_goes_round_the_platform():
    path = floor_path(-0.4, 0.0, 0.4, 0.0)
    assert len(path) == 3 and path[-1] == (0.4, 0.0)
    assert path[0] in CORNERS and path[1] in CORNERS


def test_floor_target_in_view_is_chased_directly():
    assert route(0.3, 0.3, 0.4, 0.0, 0.4, 0.0, 0.1) is None


def test_floor_target_behind_the_platform_is_reached_round_it():
    x, y, left = route(-0.4, 0.0, 0.4, 0.0, 0.4, 0.0, 0.1)
    assert (x, y) in CORNERS and left > 0.8


def test_platform_target_from_the_floor_goes_to_the_ramp():
    # From the far side the first stop is a corner, not the platform.
    x, y, _ = route(0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.2)
    assert (x, y) in CORNERS and y > 0
    # From in front of the ramp it lines up at the foot.
    x, y, _ = route(0.05, -0.5, 0.0, 0.0, 0.0, 0.0, 0.2)
    assert (x, y) == (0.0, APPROACH_Y)


def test_hunter_lined_up_at_the_foot_drives_straight_up():
    x, y, left = route(0.05, APPROACH_Y, 0.0, 0.1, 0.0, 0.1, 0.2)
    assert x == 0.05 and y > RAMP_TOP_Y and left > 0


def test_hunter_on_the_platform_chases_directly():
    assert route(0.1, 0.1, -0.1, -0.1, -0.1, -0.1, 0.2) is None


def test_hunter_comes_down_the_ramp_for_a_floor_target():
    x, y, _ = route(0.0, 0.1, 0.4, 0.4, 0.4, 0.4, 0.2)
    assert (x, y) == (0.0, TOP_Y)
    x, y, _ = route(0.0, -0.25, 0.4, 0.4, 0.4, 0.4, 0.2)
    assert (x, y) == (0.0, APPROACH_Y)


def test_follow_turns_on_the_spot_for_a_waypoint_behind():
    linear, angular, _ = follow_command(
        0, 0, 0, -0.3, 0.01, 0.5, 0.3, 1.5, 1.2, 3.0, 1.0)
    assert linear == 0.0 and angular == 1.5


def test_chase_from_behind_the_platform_climbs_the_ramp():
    # Hunter on the floor north of the platform, target on top of it.
    trail = drive_route(0.0, 0.42, -math.pi / 2, 0.05, 0.05)
    assert_no_climbing_the_edge(trail)
    hx, hy = trail[-1]
    assert level(hx, hy) == PLATFORM
    assert math.hypot(hx - 0.05, hy - 0.05) < 0.4


def test_chase_from_the_side_climbs_the_ramp():
    trail = drive_route(0.42, 0.0, math.pi, -0.1, 0.1)
    assert_no_climbing_the_edge(trail)
    assert level(*trail[-1]) == PLATFORM


def test_chase_round_the_platform_on_the_floor():
    trail = drive_route(-0.42, 0.0, 0.0, 0.42, 0.05)
    assert_no_climbing_the_edge(trail)
    assert all(level(x, y) == FLOOR for x, y in trail)
    hx, hy = trail[-1]
    assert math.hypot(hx - 0.42, hy - 0.05) < 0.35
