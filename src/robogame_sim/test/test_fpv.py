import math
import xml.etree.ElementTree as ElementTree

import pytest

from robogame_sim.robot_sdf import FPV_CAMERA, FPV_SIGHT, fpv_sight, robot_sdf

pytest.importorskip("gz.msgs.cameratrack_pb2")
from robogame_sim.fpv_toggle import fpv_track, release_track  # noqa: E402


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
