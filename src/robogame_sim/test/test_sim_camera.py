"""The Gazebo camera panel against a stand-in fpv_toggle over real
gz-transport, in a partition of its own."""

import os
import time

import pytest

pytest.importorskip("gz.transport")

from gz.msgs.stringmsg_pb2 import StringMsg  # noqa: E402
from gz.msgs.stringmsg_v_pb2 import StringMsg_V  # noqa: E402
from gz.transport import Node, NodeOptions  # noqa: E402

from robogame_sim import sim_camera  # noqa: E402


class FakeToggle:
    """Answers each view asked for on /robogame/view, as fpv_toggle does,
    but only after a moment, so an answer sent straight away is stale."""

    def __init__(self, partition):
        options = NodeOptions()
        options.partition = partition
        self.node = Node(options)
        self.state = self.node.advertise(sim_camera.STATE_TOPIC, StringMsg_V)
        self.node.subscribe(StringMsg, sim_camera.VIEW_TOPIC, self.on_view)
        self.view = ["front", "robot_1"]

    def on_view(self, message):
        self.view = (message.data.split() + ["robot_1"])[:2]
        time.sleep(0.3)
        self.send()

    def send(self):
        message = StringMsg_V()
        message.data.extend(self.view[:2] + ["robot_1", "robot_2"])
        self.state.publish(message)


def test_panel_answers_with_the_view_it_moved_to():
    partition = f"sim_camera_test_{os.getpid()}"
    toggle = FakeToggle(partition)
    camera = sim_camera.SimCamera(lambda: partition)

    # The first look at a running window says so, though the new node has
    # heard nothing yet.
    sender = __import__("threading").Thread(
        target=lambda: [time.sleep(0.5), toggle.send()], daemon=True)
    sender.start()
    first = camera.status()
    assert first["available"], first
    assert first["view"] == "front"

    moved = camera.look("fpv", "robot_2")
    assert (moved["view"], moved["robot"]) == ("fpv", "robot_2"), moved
    assert moved["robots"] == ["robot_1", "robot_2"]


def test_no_window_is_reported_after_the_wait():
    camera = sim_camera.SimCamera(lambda: f"sim_camera_none_{os.getpid()}")
    started = time.monotonic()
    reply = camera.status()
    assert not reply["available"]
    assert "no Gazebo window" in reply["error"]
    assert time.monotonic() - started < sim_camera.WAIT + 1
