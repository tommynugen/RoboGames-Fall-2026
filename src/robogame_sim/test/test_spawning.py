import math
import os
import struct

import pytest

from robogame_sim import arena_layout as al
from robogame_sim.gamepads import (
    JS_EVENT_AXIS,
    JS_EVENT_BUTTON,
    JS_EVENT_FORMAT,
    JS_EVENT_INIT,
    Gamepad,
    convert_axis,
)
from robogame_sim.spawning import CLEAR_AHEAD, clear_ahead, spawn_spot


def test_robot_facing_a_module_close_by_is_turned_away():
    # A platform 9.5 cm in front: the arm would be pressed against it.
    layout = {"size": 12, "modules": [
        {"type": "platform", "col": c, "row": r}
        for c in range(2, 9) for r in range(3, 7)]}
    ground = al.terrain(layout)
    x, y, _, yaw, moved = spawn_spot(ground, 0.04, -0.40, math.pi / 2)
    assert not moved
    assert yaw != pytest.approx(math.pi / 2)
    assert clear_ahead(ground, x, y, yaw) >= CLEAR_AHEAD


def test_robot_with_room_ahead_keeps_its_heading():
    ground = al.terrain({"size": 12, "modules": []})
    yaw = math.atan2(0.247, 0.247)
    assert spawn_spot(ground, -0.247, -0.247, yaw)[3] == yaw


def test_axes_are_scaled_like_joy_node():
    assert convert_axis(0, 0.05) == 0.0
    assert convert_axis(1000, 0.05) == 0.0
    # Up and left are positive, as joy_node has them.
    assert convert_axis(-32767, 0.05) == pytest.approx(1.0)
    assert convert_axis(32767, 0.05) == pytest.approx(-1.0)
    assert convert_axis(-32768, 0.05) == pytest.approx(1.0)


def test_gamepad_reads_joydev_events(tmp_path):
    fifo = str(tmp_path / "js")
    os.mkfifo(fifo)
    writer = os.open(fifo, os.O_RDWR)
    pad = Gamepad(fifo)

    def event(kind, number, value):
        os.write(writer, struct.pack(JS_EVENT_FORMAT, 0, value, kind, number))

    assert pad.poll() is False
    # The released trigger joydev reports first, then R2 held, Cross down.
    event(JS_EVENT_AXIS | JS_EVENT_INIT, 5, -32767)
    assert pad.poll() is True
    assert pad.axes == [0.0] * 5 + [pytest.approx(1.0)]
    event(JS_EVENT_AXIS, 5, 32767)
    event(JS_EVENT_BUTTON, 0, 1)
    pad.poll()
    assert pad.axes[5] == pytest.approx(-1.0) and pad.buttons == [1]

    pad.close()
    os.close(writer)
