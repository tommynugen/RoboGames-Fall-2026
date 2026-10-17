"""Chaser closing the loop in chase_sim, against targets that move."""

import math
import os

import pytest

from robogame_sim import chase_sim

DEFAULT_LAYOUT = os.path.join(os.path.dirname(__file__), "..", "layouts",
                              "default.json")


def wall_row(row, cols):
    return [{"type": "wall", "col": col, "row": row, "rotation": 0}
            for col in cols]


# Two rows of walls across the arena. The way from the bottom to the top
# is round the right end of the first and then up the 4 inch strip between
# the left end of the second and the edge of the floor, too narrow to turn
# in.
S_MAZE = {"size": 12, "modules": wall_row(4, range(0, 10))
          + wall_row(8, range(1, 12))}


@pytest.fixture(scope="module")
def maze():
    return chase_sim.load_world(S_MAZE)


@pytest.fixture(scope="module")
def default():
    return chase_sim.load_world(DEFAULT_LAYOUT)


def place(world, hunter, target):
    """Starts a run with the robots at the spots given, by stubbing the
    random spots it picks."""
    spots = iter([hunter, target])
    original = chase_sim.random_spot

    def fixed(*args, **kwargs):
        return next(spots)

    chase_sim.random_spot = fixed
    return original


def run_from(world, behaviour, hunter, target, seconds, trace=None):
    original = place(world, hunter, target)
    try:
        return chase_sim.run(world, behaviour, 0, seconds, trace=trace)
    finally:
        chase_sim.random_spot = original


def test_maze_reaches_a_still_target_through_the_narrow_strip(maze):
    trace = []
    result = run_from(maze, "still", (0.3, -0.45, math.pi),
                      (0.3, 0.45, 0.0), 60.0, trace)

    assert result["first_reach"] is not None
    assert result["worst_still"] < 5.0
    # It went up the strip by the edge of the floor to get there.
    assert any(x < -0.5 and abs(y - 0.25) < 0.05
               for _, x, y, *_ in trace)


def test_maze_keeps_after_a_wandering_target(maze):
    result = run_from(maze, "wander", (0.3, -0.45, math.pi),
                      (-0.2, 0.0, 0.0), 40.0)

    assert result["first_reach"] is not None
    assert result["worst_still"] < 5.0
    assert result["with_target"] > 0.6


@pytest.mark.parametrize("behaviour", ["wander", "circle", "joystick"])
def test_default_arena_stays_with_a_moving_target(default, behaviour):
    result = run_from(default, behaviour, (-0.45, -0.45, 0.0),
                      (0.45, 0.45, math.pi), 30.0)

    assert result["first_reach"] is not None
    assert result["first_reach"] < 15.0
    assert result["worst_still"] < 5.0
    assert result["with_target"] > 0.7


def test_drives_after_a_target_circling_it_rather_than_turning_on_the_spot(
        default):
    trace = []
    result = run_from(default, "circle", (-0.4, 0.4, 0.0),
                      (-0.1, 0.4, math.pi / 2), 20.0, trace)

    travelled = sum(math.dist(a[1:3], b[1:3])
                    for a, b in zip(trace, trace[1:]))
    assert result["first_reach"] is not None
    assert travelled > 1.0
    assert result["with_target"] > 0.8
