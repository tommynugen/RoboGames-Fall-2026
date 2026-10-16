import math
import os

import numpy as np
import pytest

from robogame_sim.chase import (
    chase_command,
    follow_command,
    standoff_distance,
)
from robogame_sim.terrain import (
    Terrain,
    box_top,
    corner_angle,
    next_waypoint,
    path_length,
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
    trail = [(hx, hy)]

    for _ in range(steps):
        standoff = standoff_distance(hx, hy, tx, ty, 0.0, 0.03)
        path = terrain.route(hx, hy, tx, ty, tx, ty)
        assert path is not None

        if path.partial:
            if math.dist((hx, hy), path[-1]) < 0.03:
                linear, angular = 0.0, 0.0
            else:
                linear, angular, _ = follow_command(
                    hx, hy, hyaw, *next_waypoint(path, 0.0),
                    0.3, 1.5, 1.2, 3.0, 1.0)
        elif len(path) == 2:
            linear, angular, _ = chase_command(
                hx, hy, hyaw, tx, ty, math.hypot(tx - hx, ty - hy),
                standoff, 0.3, 1.5, 1.2, 3.0, 1.0)
        else:
            linear, angular, _ = follow_command(
                hx, hy, hyaw, *next_waypoint(path, standoff),
                0.3, 1.5, 1.2, 3.0, 1.0)

        if terrain.on_slope(hx, hy):
            linear = 0.0 if linear <= 0.0 else max(linear, 0.15)

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
