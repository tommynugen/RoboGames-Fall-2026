import math
import xml.etree.ElementTree as ElementTree

import pytest

from robogame_sim.robot_sdf import FPV_CAMERA, FPV_SIGHT, fpv_sight, robot_sdf

pytest.importorskip("gz.msgs.cameratrack_pb2")
from robogame_sim.fpv_toggle import (  # noqa: E402
    KEY_F1,
    VIEWS,
    fpv_track,
    key_action,
    release_track,
    view_camera,
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
def test_every_view_looks_at_the_middle_of_the_arena_from_above(view):
    pose = view_camera(view).pose
    p = pose.position
    dx, dy, dz = rotate(pose.orientation, (1.0, 0.0, 0.0))
    assert dz < 0
    # Where the line of sight meets the floor.
    s = -p.z / dz
    assert math.hypot(p.x + s * dx, p.y + s * dy) < 0.1
    # The arena's front edge at the bottom of the picture: up on screen is
    # away from -y.
    _, up_y, _ = rotate(pose.orientation, (0.0, 0.0, 1.0))
    assert up_y > 0
