import math
import threading
import xml.etree.ElementTree as ElementTree
from types import SimpleNamespace

import pytest

from robogame_sim.robot_sdf import FPV_CAMERA, FPV_SIGHT, fpv_sight, robot_sdf

pytest.importorskip("gz.msgs.cameratrack_pb2")
from robogame_sim.fpv_toggle import (  # noqa: E402
    ARENA_CORNERS,
    ARENA_HALF,
    FILL,
    HFOV,
    KEY_ESCAPE,
    KEY_F1,
    START_VIEW,
    VIEWS,
    FpvToggle,
    camera_axes,
    dot,
    fpv_track,
    key_action,
    release_track,
    view_camera,
    view_pose,
)


def chassis_visuals(sdf):
    root = ElementTree.fromstring(sdf)
    chassis = next(link for link in root.iter("link")
                   if link.get("name") == "chassis")
    return {visual.get("name") for visual in chassis.iter("visual")}


def test_fpv_adds_lens_and_sight_to_the_chassis_only_when_asked():
    assert {"fpv_lens", "fpv_sight"} <= chassis_visuals(robot_sdf(2, True))
    assert not {"fpv_lens", "fpv_sight"} & chassis_visuals(robot_sdf(2))


def test_sight_is_out_along_the_cameras_line():
    x, y, z, pitch = FPV_CAMERA
    sx, sy, sz = fpv_sight()
    assert math.isclose(math.dist((x, y, z), (sx, sy, sz)), FPV_SIGHT)
    assert math.isclose(math.atan2(z - sz, sx - x), pitch)


def test_view_follows_the_lens_and_looks_at_the_sight():
    message = fpv_track("robot_2")
    assert message.follow_target.name == "robot_2"
    # The scene names the visual model::link::visual.
    assert message.track_target.name == "robot_2::chassis::fpv_sight"
    offset = message.follow_offset
    assert (offset.x, offset.y, offset.z) == pytest.approx(FPV_CAMERA[:3])


def test_release_names_a_target_no_robot_has():
    message = release_track()
    assert message.follow_target.name and message.track_target.name
    assert "robot_" not in message.follow_target.name


def test_number_keys_pick_views_and_4_toggles_fpv():
    assert key_action(ord("1")) == ("view", "top")
    assert key_action(ord("2")) == ("view", "front")
    assert key_action(ord("3")) == ("view", "front_right")
    assert key_action(ord("4")) == ("fpv", None)
    assert key_action(KEY_F1 + 2) == ("robot", "robot_3")
    assert key_action(ord("F")) is None
    assert key_action(ord("5")) is None


def rotate(q, v):
    """v turned by the unit quaternion q."""
    w, x, y, z = q.w, q.x, q.y, q.z
    vx, vy, vz = v
    # t = 2 q.xyz cross v, then v + w t + q.xyz cross t.
    tx, ty, tz = (2 * (y * vz - z * vy), 2 * (z * vx - x * vz),
                  2 * (x * vy - y * vx))
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


@pytest.mark.parametrize("view", list(VIEWS))
def test_every_view_looks_down_on_the_arena_front_edge_at_the_bottom(view):
    pose = view_camera(view).pose
    p = pose.position
    dx, dy, dz = rotate(pose.orientation, (1.0, 0.0, 0.0))
    assert dz < 0
    # Where the line of sight meets the floor.
    s = -p.z / dz
    assert abs(p.x + s * dx) < ARENA_HALF and abs(p.y + s * dy) < ARENA_HALF
    # The arena's front edge at the bottom of the picture: up on screen is
    # away from -y.
    _, up_y, _ = rotate(pose.orientation, (0.0, 0.0, 1.0))
    assert up_y > 0


@pytest.mark.parametrize("aspect", [577 / 797, 1.0, 16 / 9])
@pytest.mark.parametrize("view", list(VIEWS))
def test_every_view_fits_the_whole_arena_and_no_more(view, aspect):
    x, y, z, _, pitch, yaw = view_pose(view, aspect)
    forward, left, up = camera_axes(pitch, yaw)
    across = math.tan(HFOV / 2)
    upward = across / aspect
    sides, highs = [], []
    for p in ARENA_CORNERS:
        offset = (p[0] - x, p[1] - y, p[2] - z)
        ahead = dot(offset, forward)
        sides.append(dot(offset, left) / ahead / across)
        highs.append(dot(offset, up) / ahead / upward)
    assert max(map(abs, sides + highs)) <= FILL + 1e-6
    # As close as it fits: the arena spans the picture across or up, and
    # sits in the middle of it.
    across_span = max(sides) - min(sides)
    up_span = max(highs) - min(highs)
    assert max(across_span, up_span) == pytest.approx(2 * FILL, abs=1e-6)
    assert max(sides) + min(sides) == pytest.approx(0, abs=1e-6)
    assert max(highs) + min(highs) == pytest.approx(0, abs=1e-6)


class FakeToggle(FpvToggle):
    """FpvToggle without gz-transport: records what it would move to."""

    def __init__(self):
        self.robots = ["robot_1", "robot_2"]
        self.has_fpv = True
        self.robot = "robot_1"
        self.current = START_VIEW
        self.aspect = 1.0
        self.lock = threading.Lock()
        self.moves = []
        self.requests = SimpleNamespace(
            put=lambda request: self.moves.append("fly"))
        self.track = SimpleNamespace(
            publish=lambda message: self.moves.append(
                message.follow_target.name))
        self.state = SimpleNamespace(publish=lambda message: None)

    def press(self, key):
        self.on_key(SimpleNamespace(data=key))
        moves, self.moves = self.moves, []
        return moves


def test_a_key_switches_to_its_view_and_again_does_nothing():
    toggle = FakeToggle()
    assert toggle.press(ord("1")) == ["fly"]
    assert toggle.press(ord("1")) == []
    assert toggle.press(ord("4")) == ["robot_1"]
    assert toggle.press(ord("4")) == []
    assert toggle.press(KEY_F1 + 1) == ["robot_2"]
    assert toggle.press(KEY_F1 + 1) == []
    # Out of FPV: let go of the robot, then fly.
    assert toggle.press(ord("3")) == ["fpv_off", "fly"]
    assert toggle.current == "front_right"


def test_after_escape_the_same_view_key_moves_the_camera_again():
    toggle = FakeToggle()
    toggle.press(ord("4"))
    toggle.press(KEY_ESCAPE)
    assert toggle.press(ord("4")) == ["robot_1"]
