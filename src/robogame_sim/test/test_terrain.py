import math
import os

import numpy as np
import pytest

from robogame_sim.chase import Chaser
from robogame_sim.terrain import (
    CLEARANCE,
    HALF_LENGTH,
    RESOLUTION,
    TURN_CLEARANCE,
    Terrain,
    box_top,
    corner_angle,
    line_axis,
    next_waypoint,
    path_length,
    point_along,
)

HEIGHT = 0.0254
HALF = 0.1905
FOOT_Y = -0.3175

DEFAULT_LAYOUT = os.path.join(os.path.dirname(__file__), "..", "layouts",
                              "default.json")


def arena():
    """Like the CAD arena: a floor, a platform in the middle and a ramp
    along its -y side, rising toward +y."""
    floor = box_top((1.14, 1.14, 0.0)) + [0, 0, 0]
    platform = box_top((2 * HALF, 2 * HALF, HEIGHT)) + [0, 0, HEIGHT / 2]
    low_l, low_r = (-HALF, FOOT_Y, 0.0), (HALF, FOOT_Y, 0.0)
    top_l, top_r = (-HALF, -HALF, HEIGHT), (HALF, -HALF, HEIGHT)
    ramp = np.array([[low_l, low_r, top_r], [low_l, top_r, top_l]])
    return Terrain(np.concatenate([floor, platform, ramp]))


@pytest.fixture(scope="module")
def terrain():
    return arena()


def on_platform(terrain, x, y):
    return terrain.height_at(x, y) > HEIGHT - 0.002


def drive(terrain, hx, hy, hyaw, tx, ty, steps=800, dt=0.05):
    """Drives a hunter after a still target the way chase_node does, and
    returns the hunter's centre at every step."""
    chaser = Chaser(terrain)
    trail = [(hx, hy)]

    for step in range(steps):
        chaser.track(tx, ty, step * dt)
        linear, angular = chaser.update(step * dt, hx, hy, hyaw)
        assert chaser.path is not None

        hx += linear * dt * math.cos(hyaw)
        hy += linear * dt * math.sin(hyaw)
        hyaw += angular * dt
        trail.append((hx, hy))

    return trail


def assert_never_climbs_an_edge(terrain, trail):
    """Between steps the ground under the hunter changes smoothly, and the
    hunter's centre stays off the steps."""
    for (ax, ay), (bx, by) in zip(trail, trail[1:]):
        assert abs(terrain.height_at(bx, by) - terrain.height_at(ax, ay)) \
            < 0.01, (ax, ay, bx, by)
        assert terrain.clearance[terrain.cell(bx, by)] > 0.03, (bx, by)


def test_height_map_has_the_platform_and_ramp(terrain):
    assert abs(terrain.height_at(0.0, 0.0) - HEIGHT) < 1e-6
    assert abs(terrain.height_at(0.4, 0.0)) < 1e-6
    assert 0.005 < terrain.height_at(0.0, -0.25) < 0.02
    assert math.isnan(terrain.height_at(0.7, 0.0))


def test_platform_edges_are_not_driveable_but_the_ramp_is(terrain):
    assert not terrain.free[terrain.cell(HALF, 0.0)]
    assert not terrain.free[terrain.cell(0.0, HALF)]
    assert not terrain.free[terrain.cell(HALF, -0.25)]
    assert terrain.free[terrain.cell(0.0, -0.25)]
    assert terrain.free[terrain.cell(0.0, FOOT_Y)]


def test_ramp_must_be_driven_along_its_slope(terrain):
    cell = terrain.cell(0.0, -0.25)
    assert terrain.aligned(cell, 0.0, 1.0)
    assert terrain.aligned(cell, 0.0, -1.0)
    assert not terrain.aligned(cell, 1.0, 1.0)
    assert terrain.aligned(terrain.cell(0.4, 0.4), 1.0, 1.0)


def test_clear_floor_is_a_straight_run(terrain):
    path = terrain.route(0.4, 0.4, 0.4, -0.4, 0.4, -0.4)
    assert len(path) == 2


def test_platform_in_the_way_is_gone_round(terrain):
    path = terrain.route(-0.42, 0.0, 0.42, 0.05, 0.42, 0.05)
    assert len(path) > 2
    for x, y in path:
        assert not on_platform(terrain, x, y)
    # Round the near side, not the long way past the ramp.
    assert all(y > 0.0 for _, y in path[1:-1])


def test_target_on_the_platform_is_reached_up_the_ramp(terrain):
    path = terrain.route(0.0, 0.42, 0.05, 0.05, 0.05, 0.05)
    assert len(path) > 2
    # The leg onto the platform starts in front of the ramp's foot.
    foot = [(x, y) for x, y in path if y < FOOT_Y]
    assert foot and abs(foot[-1][0]) < HALF


def test_next_waypoint_slows_for_the_turn_at_the_ramp():
    path = [(0.3, -0.4), (0.0, -0.4), (0.0, 0.0)]
    x, y, left = next_waypoint(path, 0.2)
    assert (x, y) == (0.0, -0.4) and abs(left - 0.33) < 1e-9
    assert corner_angle(*path) == pytest.approx(math.pi / 2)
    assert path_length(path) == pytest.approx(0.7)


@pytest.mark.parametrize("start, target", [
    ((0.0, 0.42, -math.pi / 2), (0.05, 0.05)),
    ((0.42, 0.0, math.pi), (-0.1, 0.1)),
    ((-0.42, 0.3, 0.0), (0.1, -0.05)),
])
def test_chase_onto_the_platform_climbs_the_ramp(terrain, start, target):
    trail = drive(terrain, *start, *target)
    assert_never_climbs_an_edge(terrain, trail)
    # Up on the platform, or on the ramp if the target is near its top.
    hx, hy = trail[-1]
    assert terrain.height_at(hx, hy) > 0.005
    assert math.dist((hx, hy), target) < 0.36


def test_chase_round_the_platform_on_the_floor(terrain):
    trail = drive(terrain, -0.42, 0.0, 0.0, 0.42, 0.05)
    assert_never_climbs_an_edge(terrain, trail)
    assert all(not on_platform(terrain, x, y) for x, y in trail)
    assert math.dist(trail[-1], (0.42, 0.05)) < 0.36


def test_chase_off_the_platform_comes_down_the_ramp(terrain):
    trail = drive(terrain, 0.0, 0.05, 0.0, 0.3, 0.42)
    assert_never_climbs_an_edge(terrain, trail)
    assert not on_platform(terrain, *trail[-1])
    assert math.dist(trail[-1], (0.3, 0.42)) < 0.36


def test_default_arena_platform_target_is_reached_up_the_ramp():
    from robogame_sim import arena_layout

    ground = arena_layout.terrain(arena_layout.load(DEFAULT_LAYOUT))
    trail = drive(ground, 0.0, 0.42, -math.pi / 2, 0.05, 0.05)
    assert_never_climbs_an_edge(ground, trail)
    assert on_platform(ground, *trail[-1])


def walls(*boxes, floor=0.6):
    """A floor with walls 0.05 high on it, each (x0, y0, x1, y1)."""
    parts = [box_top((2 * floor, 2 * floor, 0.0))]
    for x0, y0, x1, y1 in boxes:
        parts.append(box_top((x1 - x0, y1 - y0, 0.05))
                     + [(x0 + x1) / 2, (y0 + y1) / 2, 0.025])
    return Terrain(np.concatenate(parts))


INCH = 0.0254


def test_cell_centres_are_on_the_half_inch_lattice(terrain):
    x, y = terrain.centre(terrain.cell(0.1, -0.2))
    assert x / RESOLUTION == pytest.approx(round(x / RESOLUTION))
    assert y / RESOLUTION == pytest.approx(round(y / RESOLUTION))


def test_clearance_is_measured_from_the_robot_itself():
    ground = walls((-0.6, 0.1, 0.6, 0.1 + INCH))
    wall, drop = ground.room_at(0.0, 0.1 - 0.05)
    assert wall == pytest.approx(0.05, abs=0.002) and drop == math.inf
    assert not ground.can_turn(0.0, 0.1 - 0.05)
    assert ground.can_turn(0.0, 0.1 - TURN_CLEARANCE - 0.003)


def test_the_floor_edge_is_a_drop_the_ends_may_hang_over():
    ground = walls()
    # A wall this close would leave no room to turn.
    x = 0.6 - 0.052
    wall, drop = ground.room_at(x, 0.0)
    assert wall == math.inf and drop == pytest.approx(0.052, abs=0.002)
    assert ground.can_turn(x, 0.0)


def gap(width):
    """Two walls across the arena at y 0 with a gap width wide in the
    middle, so the way from one side to the other is through it."""
    return walls((-0.6, -INCH / 2, -width / 2, INCH / 2),
                 (width / 2, -INCH / 2, 0.6, INCH / 2))


def test_gap_narrower_than_a_turn_is_driven_through_lined_up():
    ground = gap(4.5 * INCH)
    middle = ground.cell(0.0, 0.0)
    assert not ground.free[middle]
    # It has room lined up with the gap, along y.
    assert ground.lined_up[2][middle]

    path = ground.route(0.2, -0.3, 0.2, 0.3, 0.2, 0.3)
    assert path is not None and not path.partial
    assert len(path) > 2

    # Every leg through the gap runs along it.
    for a, b in zip(path, path[1:]):
        cells = ground.line_cells(*a, *b)
        if not ground.free[cells].all():
            assert line_axis(b[0] - a[0], b[1] - a[1]) == 2


def test_gap_narrower_than_the_robot_is_not_driven_through():
    ground = gap(2 * CLEARANCE - 0.01)
    path = ground.route(0.2, -0.3, 0.2, 0.3, 0.2, 0.3)
    assert path.partial
    assert path[-1][1] < 0.0


def test_chase_through_a_narrow_gap(terrain=None):
    ground = gap(4.5 * INCH)
    trail = drive(ground, 0.3, -0.3, math.pi, 0.25, 0.35, steps=600)
    assert math.dist(trail[-1], (0.25, 0.35)) < 0.25
    # Through the gap, never closer to a wall than its body allows.
    for x, y in trail:
        if abs(y) < 0.05:
            assert abs(x) < 4.5 * INCH / 2 - 0.03


def test_point_along_a_path():
    path = [(0.0, 0.0), (0.1, 0.0), (0.1, 0.2)]
    assert point_along(path, 0.05) == pytest.approx((0.05, 0.0))
    assert point_along(path, 0.2) == pytest.approx((0.1, 0.1))
    assert point_along(path, 1.0) == (0.1, 0.2)


def test_hunter_front_is_its_wheels_with_the_arm_stowed():
    assert HALF_LENGTH == pytest.approx(0.05)
