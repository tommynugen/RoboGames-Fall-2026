import json
import math
import os
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import numpy as np
import pytest

from robogame_sim import arena_layout as al
from robogame_sim.gamemaster import handler
from robogame_sim.terrain import Terrain, read_stl


def placed(layout):
    """A layout's modules but the floor tiles, which a layout without a
    base gets on every square."""
    return [m for m in layout["modules"] if m["type"] != "floor"]
from robogame_sim.terrain import parse_stl as read_stl_bytes

HERE = os.path.dirname(__file__)
PACKAGE = os.path.join(HERE, "..")
CAD = os.path.join(PACKAGE, "models", al.CAD_MODEL, "model.sdf")
WORLD = os.path.join(PACKAGE, "worlds", "robogame_arena.sdf")


def platform(col, row, kind="platform"):
    return {"type": kind, "col": col, "row": row}


def ramp(col, row, rotation):
    return {"type": "ramp", "col": col, "row": row, "rotation": rotation}


def side(kind, col, row, rotation):
    return {"type": kind, "col": col, "row": row, "rotation": rotation}


def test_module_size_is_the_cads():
    lo, hi = al.mesh_bounds(al.FLOOR.mesh)
    assert np.allclose(hi[:2] - lo[:2], al.MODULE, atol=1e-4)

    # The CAD's floor tiles cover the same square as a layout's.
    built = al.terrain({"size": al.SIZE, "modules": []})
    cad = Terrain.from_sdf(CAD)
    assert built.shape == cad.shape
    assert np.allclose(built.origin, cad.origin, atol=1e-3)


def test_default_layout_has_no_problems():
    layout = al.load(os.path.join(PACKAGE, "layouts", "default.json"))
    assert layout["size"] == al.SIZE
    assert al.check(layout) == []


def test_every_module_has_a_mesh():
    for part in (al.FLOOR, *al.PARTS.values()):
        assert len(read_stl(os.path.join(al.mesh_dir(), part.mesh)))


@pytest.mark.parametrize("rotation", al.ROTATIONS)
def test_ramps_rise_the_way_they_are_turned(rotation):
    layout = {"size": 12, "modules": [ramp(5, 5, rotation)]}
    ground = al.terrain(layout)
    x, y = al.centre(layout, 5, 5)
    ux, uy = al.uphill(rotation)
    low = ground.height_at(x - 0.04 * ux, y - 0.04 * uy)
    high = ground.height_at(x + 0.04 * ux, y + 0.04 * uy)
    assert high - low > 0.01


@pytest.mark.parametrize("kind,height", [("wall", 0.0508),
                                         ("thwomp", 0.1524)])
@pytest.mark.parametrize("rotation", al.ROTATIONS)
def test_walls_stand_on_the_peg_line_they_are_turned_to(kind, height,
                                                        rotation):
    layout = {"size": 12, "modules": [side(kind, 5, 5, rotation)]}
    ground = al.terrain(layout)
    x, y = al.centre(layout, 5, 5)
    ux, uy = al.uphill(rotation)
    # Across the peg line: floor, the wall, floor again.
    across = [ground.height_at(x + d * ux, y + d * uy)
              for d in (-0.03, 0.0, al.PEG_LINE, 0.045)]
    assert across[0] == pytest.approx(0.0, abs=1e-3)
    assert across[1] == pytest.approx(0.0, abs=1e-3)
    assert across[2] == pytest.approx(height, abs=1e-3)
    assert across[3] == pytest.approx(0.0, abs=1e-3)
    # Along it, the whole width of the square.
    vx, vy = -uy, ux
    for d in (-0.04, 0.04):
        z = ground.height_at(x + al.PEG_LINE * ux + d * vx,
                             y + al.PEG_LINE * uy + d * vy)
        assert z == pytest.approx(height, abs=1e-3)


def test_platforms_are_their_height():
    layout = {"size": 12, "modules": [platform(2, 2),
                                      platform(8, 8, "platform_2")]}
    ground = al.terrain(layout)
    assert ground.height_at(*al.centre(layout, 2, 2)) == \
        pytest.approx(0.0254, abs=1e-3)
    assert ground.height_at(*al.centre(layout, 8, 8)) == \
        pytest.approx(0.0508, abs=1e-3)


def test_turned_ramp_leads_onto_its_platform():
    # Platform west of the ramp, so the ramp rises toward -x.
    layout = {"size": 12, "modules": [
        platform(2, 4), platform(2, 5), platform(1, 4), platform(1, 5),
        ramp(3, 4, 90), ramp(3, 5, 90)]}
    assert al.check(layout) == []
    ground = al.terrain(layout)
    x, y = al.centre(layout, 3, 4)
    cell = ground.cell(x, y + al.MODULE / 2)
    ax, ay = ground.axis[cell]
    assert abs(ax) > 0.99 and abs(ay) < 0.01


def test_malformed_layouts_are_refused():
    for bad in ({"modules": [platform(12, 0)]},
                {"modules": [platform(1, 1), ramp(1, 1, 0)]},
                {"modules": [ramp(1, 1, 45)]},
                {"modules": [side("wall", 1, 1, 30)]},
                {"modules": [{"type": "tower", "col": 1, "row": 1}]}):
        with pytest.raises(ValueError):
            al.normalise(bad)


def test_rotation_is_kept_only_for_modules_that_turn():
    layout = al.normalise({"modules": [
        {"type": "platform_2", "col": 1, "row": 1, "rotation": 90},
        side("thwomp", 2, 1, 450)]})
    assert layout["size"] == al.SIZE
    assert placed(layout) == [platform(1, 1, "platform_2"),
                                 side("thwomp", 2, 1, 90)]


def test_checks_find_ramps_and_platforms_robots_cannot_use():
    layout = {"size": 12, "modules": [
        platform(1, 7), platform(1, 6), platform(2, 7), platform(2, 6),
        ramp(0, 7, 270),                # low end against the arena edge
        ramp(6, 2, 0),                  # leads to nothing
        platform(9, 9),                 # a lone module, no ramp to it
        ramp(5, 5, 0), platform(5, 4),  # low end runs into a platform
        ramp(8, 4, 0), side("wall", 8, 3, 0),  # into a wall facing it
        ramp(10, 4, 0), platform(10, 5, "platform_2"),  # onto a 2" one
    ]}
    found = {}
    for p in al.check(layout):
        found[(p["col"], p["row"])] = found.get((p["col"], p["row"]), "") \
            + p["message"]

    assert "edge of the arena" in found[(0, 7)]
    assert "no platform" in found[(6, 2)]
    assert "no robot can get onto" in found[(9, 9)]
    assert "runs into a 1\" platform" in found[(5, 5)]
    assert "runs into a wall" in found[(8, 4)]
    assert "2\" platform" in found[(10, 4)]
    assert "no robot can get onto" in found[(1, 7)]
    # 2" platforms are obstacles, not somewhere to get to.
    assert (10, 5) not in found


def test_only_a_wall_beside_a_ramp_runs_into_it():
    layout = {"size": 12, "modules": [
        ramp(5, 5, 0), ramp(6, 5, 0),
        platform(5, 6), platform(6, 6), platform(5, 7), platform(6, 7),
        side("wall", 5, 4, 180), side("thwomp", 6, 4, 270)]}
    assert not [p for p in al.check(layout) if "runs into" in p["message"]]

    # Three inches is too little room to line up with the ramp, though,
    # which the terrain finds.
    layout["modules"][-2:] = [side("wall", 5, 3, 180),
                              side("thwomp", 6, 3, 270)]
    assert al.check(layout) == []


def test_world_has_the_layout_instead_of_the_cad_include():
    layout = {"size": 12, "modules": [
        platform(5, 2), platform(6, 2), platform(5, 3), platform(6, 3),
        side("wall", 0, 0, 180)]}
    text = al.world_sdf(layout, WORLD)

    assert "<uri>model://robogame_arena_cad</uri>" not in text
    assert 'collision name="platform_5_3"' in text
    assert 'collision name="wall_0_0"' in text
    assert "meshes/wall_module.stl" in text
    assert text.count('<collision name="floor_') == 144
    # token_4 sits at (0, -0.3), now on the platform at cols 5 and 6.
    z = float(text.split('<model name="token_4">')[1]
              .split("<pose>")[1].split()[2])
    assert z > 0.0254


def test_layouts_save_and_load(tmp_path):
    layout = {"size": 12, "modules": [ramp(3, 2, 180), platform(3, 1),
                                      side("wall", 4, 4, 270)]}
    path = al.save("trial", layout, folder=str(tmp_path))
    assert al.load(path) == al.normalise(layout)
    saved = placed(json.loads(open(path).read()))
    assert [m.get("rotation") for m in saved] == [None, 180, 270]
    with pytest.raises(ValueError):
        al.save("../escape", layout, folder=str(tmp_path))


@pytest.fixture
def editor(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBOGAME_LAYOUTS", str(tmp_path))
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler())
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{server.server_address[1]}", tmp_path
    server.shutdown()
    server.server_close()


def call(url, method="GET", body=None):
    data = json.dumps(body).encode() if body is not None else None
    request = urllib.request.Request(url, data=data, method=method)
    try:
        with urllib.request.urlopen(request) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as error:
        return error.code, error.read()


def test_editor_serves_saves_and_checks(editor):
    url, folder = editor
    status, page = call(url + "/")
    assert status == 200 and b"Arena editor" in page and b"Robot console" in page
    status, page = call(url + "/editor")
    assert status == 200 and b"Arena layout" in page
    # Without a console node the console tab says why.
    status, page = call(url + "/console")
    assert status == 200 and b"ROS is not sourced" in page
    assert call(url + "/api/state")[0] == 404
    status, script = call(url + "/arena_draw.js")
    assert status == 200 and b"function drawModule" in script
    status, script = call(url + "/arena_view3d.js")
    assert status == 200 and b"export function create" in script
    status, mesh = call(url + "/api/meshes/ramp_module_curved.stl")
    assert status == 200 and len(mesh) > 84
    # Only the arena's meshes, nothing else on the disk.
    assert call(url + "/api/meshes/..%2F..%2Fsetup.py")[0] == 404
    assert call(url + "/api/meshes/model.sdf")[0] == 404

    status, body = call(url + "/api/modules")
    modules = json.loads(body)
    assert status == 200 and modules["size"] == al.SIZE
    assert set(modules["types"]) == set(al.TYPES)
    assert modules["types"]["thwomp"]["height"] == pytest.approx(0.1524,
                                                                 abs=1e-3)

    layout = {"size": 12, "modules": [
        ramp(5, 4, 0), ramp(6, 4, 0), platform(5, 5), platform(6, 5),
        platform(5, 6), platform(6, 6), side("thwomp", 0, 0, 90)]}
    status, body = call(url + "/api/layouts/mine", "PUT", layout)
    assert status == 200
    assert os.path.exists(folder / "mine.json")
    assert json.loads(body)["problems"] == []

    status, body = call(url + "/api/layouts")
    assert "mine" in json.loads(body)["layouts"]

    status, body = call(url + "/api/layouts/mine")
    assert json.loads(body) == al.normalise(layout)

    status, body = call(url + "/api/check", "POST",
                        {"size": 12, "modules": [ramp(0, 0, 180)]})
    assert status == 200 and json.loads(body)["problems"]

    # Spaces are - in the file name.
    status, body = call(url + "/api/layouts/my%20%20arena", "PUT", layout)
    assert status == 200 and json.loads(body)["name"] == "my-arena"
    assert json.loads(call(url + "/api/layouts/my%20arena")[1]) == \
        al.normalise(layout)
    assert call(url + "/api/layouts/bad%2Fname", "PUT", layout)[0] == 400
    assert call(url + "/api/layouts/nothere")[0] == 404
    assert call(url + "/api/check", "POST",
                {"modules": [ramp(1, 1, 45)]})[0] == 400


def test_chase_climbs_a_turned_ramp():
    from test_terrain import assert_never_climbs_an_edge, drive

    layout = {"size": 12, "modules": [
        platform(c, r) for c in (1, 2, 3, 4) for r in (5, 6, 7)]
        + [ramp(5, r, 90) for r in (5, 6, 7)]}
    ground = al.terrain(layout)
    # Far enough onto the platform that the hunter cannot reach it from
    # the floor in front of the ramp.
    target = al.centre(layout, 1, 6)
    trail = drive(ground, 0.3, -0.3, math.pi / 2, *target)
    assert_never_climbs_an_edge(ground, trail)
    assert ground.height_at(*trail[-1]) > 0.02


def test_chase_goes_round_a_wall():
    from test_terrain import assert_never_climbs_an_edge, drive

    # A row of walls across the arena with a gap at the east end.
    layout = {"size": 12, "modules": [
        side("wall", c, 5, 0) for c in range(9)]}
    ground = al.terrain(layout)
    trail = drive(ground, -0.3, -0.2, math.pi / 2, -0.3, 0.3)
    assert_never_climbs_an_edge(ground, trail)
    assert max(x for x, _ in trail) > al.centre(layout, 8, 5)[0]
    assert math.dist(trail[-1], (-0.3, 0.3)) < 0.36


def test_unreachable_target_is_waited_for_beside_its_platform():
    from test_terrain import assert_never_climbs_an_edge, drive

    layout = {"size": 12, "modules": [
        platform(7, 7, "platform_2"), platform(7, 6, "platform_2")]}
    ground = al.terrain(layout)
    target = al.centre(layout, 7, 7)

    path = ground.route(-0.3, -0.3, *target, *target)
    assert path.partial
    assert abs(ground.height_at(*path[-1])) < 0.001

    trail = drive(ground, -0.3, -0.3, 0.0, *target)
    assert_never_climbs_an_edge(ground, trail)
    end = trail[-1]
    assert abs(ground.height_at(*end)) < 0.001
    # Right beside the platform, and parked there.
    assert math.dist(end, target) < 0.16
    assert math.dist(end, trail[-20]) < 1e-3


def test_spawn_spot_in_a_pocket_is_moved_to_open_floor():
    # The corner between two banks of ramps, where the spawn circle puts
    # robot 1, with a thwomp beside it: like the -x, -y quarter of a
    # layout with a ramp on every side of the middle platform.
    layout = {"size": 12, "modules": [
        ramp(3, r, 270) for r in (4, 5, 6, 7)] + [
        ramp(c, 3, 0) for c in (4, 5, 6, 7)] + [
        platform(c, r) for c in (4, 5, 6, 7) for r in (4, 5, 6, 7)] + [
        side("thwomp", 1, 3, 270)]}
    ground = al.terrain(layout)
    floor = al.main_floor(ground)

    x, y = al.open_spot(ground, -0.247, -0.247)
    assert (x, y) != (-0.247, -0.247)
    cell = ground.cell(x, y)
    assert floor[cell] and ground.clearance[cell] >= 0.08
    assert not ground.axis[cell].any(), "on or by a ramp"
    assert math.dist((x, y), (-0.247, -0.247)) < 0.15

    # Kept apart from a robot already there.
    other = al.open_spot(ground, -0.247, -0.247, taken=[(x, y)])
    assert math.dist(other, (x, y)) >= 0.2


def test_open_spawn_spot_is_kept():
    ground = al.terrain({"size": 12, "modules": []})
    assert al.open_spot(ground, -0.247, -0.247) == (-0.247, -0.247)
    assert al.open_spot(ground, 0.247, 0.247,
                        taken=[(-0.247, -0.247)]) == (0.247, 0.247)


def test_default_layout_spawns_clear_of_the_platform_corners():
    # The spawn circle passes 2 to 3 cm from the corners of the middle
    # platform.
    layout = al.load(os.path.join(PACKAGE, "layouts", "default.json"))
    ground = al.terrain(layout)
    floor = al.main_floor(ground)
    taken = []

    for spot in ((-0.247, -0.247), (0.247, 0.247)):
        x, y = al.open_spot(ground, *spot, taken)
        taken.append((x, y))
        cell = ground.cell(x, y)
        assert floor[cell] and ground.clearance[cell] >= 0.08
        assert not ground.axis[cell].any(), "on or by a ramp"
        assert math.dist((x, y), spot) < 0.15


def on(module, level):
    return dict(module, level=level)


# A 1" platform reached by ramps from the floor, with ramps on it up to a
# 2" platform. Banks of three ramps, as the planner cannot line up with
# two.
TWO_STOREYS = (
    [ramp(c, 2, 0) for c in (2, 3, 4)]
    + [platform(c, r) for c in range(2, 9) for r in range(3, 7)]
    + [on(ramp(c, 6, 0), 1) for c in (4, 5, 6)]
    + [platform(c, r, "platform_2") for c in range(3, 8)
       for r in (7, 8, 9)])


def test_stacked_modules_need_a_platform_under_them():
    for bad in ({"modules": [on(platform(1, 1), 1)]},
                {"modules": [ramp(1, 1, 0), on(platform(1, 1), 1)]},
                {"modules": [side("wall", 1, 1, 0), on(ramp(1, 1, 0), 1)]},
                {"modules": [platform(1, 1), on(ramp(1, 1, 0), 2)]},
                {"modules": [platform(1, 1), platform(1, 1)]},
                {"modules": [on(platform(1, 1), -1)]}):
        with pytest.raises(ValueError):
            al.normalise(bad)

    layout = al.normalise({"modules": [
        on(side("thwomp", 1, 1, 90), 2), platform(1, 1, "platform_2"),
        on(platform(1, 1), 1), platform(2, 1)]})
    # Bottom first, and level left out on the floor.
    assert placed(layout) == [
        platform(1, 1, "platform_2"), on(platform(1, 1), 1),
        on(side("thwomp", 1, 1, 90), 2), platform(2, 1)]


def test_stacked_modules_stand_on_the_platforms_under_them():
    layout = {"size": 12, "modules": [
        platform(2, 2), on(platform(2, 2), 1),
        platform(5, 5, "platform_2"), on(ramp(5, 5, 0), 1),
        platform(8, 8), on(side("wall", 8, 8, 0), 1)]}
    ground = al.terrain(layout)
    inch = 0.0254

    assert ground.height_at(*al.centre(layout, 2, 2)) == \
        pytest.approx(2 * inch, abs=1e-3)

    # The curved ramp's foot, at the module's edge, is on the platform.
    x, y = al.centre(layout, 5, 5)
    assert ground.height_at(x, y - 0.05) == pytest.approx(2 * inch,
                                                           abs=2e-3)
    assert ground.height_at(x, y + 0.04) == pytest.approx(3 * inch,
                                                           abs=2e-3)

    x, y = al.centre(layout, 8, 8)
    assert ground.height_at(x, y + al.PEG_LINE) == \
        pytest.approx(3 * inch, abs=1e-3)
    assert ground.height_at(x, y - 0.03) == pytest.approx(inch, abs=1e-3)

    text = al.world_sdf(layout, WORLD)
    assert 'collision name="platform_2_2_1"' in text
    assert 'collision name="ramp_5_5_1"' in text


def test_ramp_on_a_platform_leads_up_to_a_higher_one():
    layout = {"size": 12, "modules": list(TWO_STOREYS)}
    assert al.check(layout) == []

    # Without the ramps on top the 2" platform is an obstacle again.
    lower = [m for m in TWO_STOREYS if m.get("level") != 1]
    assert al.check({"size": 12, "modules": lower}) == []


def test_checks_compare_heights_across_levels():
    layout = {"size": 12, "modules": [
        # On a 1" platform, its low end over the edge of it.
        platform(2, 2), platform(3, 2), on(ramp(2, 2, 0), 1),
        on(ramp(3, 2, 0), 1), platform(2, 3), platform(3, 3),
        # From the floor into a stack two platforms high.
        ramp(8, 2, 0), platform(8, 3), on(platform(8, 3), 1)]}
    found = {}
    for p in al.check(layout):
        found.setdefault((p["col"], p["row"]), []).append(p["message"])

    assert any("an inch above the ground" in m for m in found[(2, 2)])
    assert any("an inch above the top of the ramp" in m
               for m in found[(8, 2)])


def test_ramps_chain_up_and_turn_on_one_module_landings():
    layout = {"size": 12, "modules": [
        # Floor to 1" to a 2" corner, the second ramp on a platform.
        ramp(0, 2, 180), platform(0, 1), on(ramp(0, 1, 180), 1),
        platform(0, 0, "platform_2"),
        # Up toward -x onto a one module landing 3" up, then turning up
        # toward +y onto a 4" platform.
        ramp(7, 4, 90), platform(6, 4), on(ramp(6, 4, 90), 1),
        platform(5, 4, "platform_2"), on(ramp(5, 4, 90), 1),
        platform(4, 4, "platform_2"), on(platform(4, 4), 1),
        platform(4, 5, "platform_2"), on(platform(4, 5), 1),
        on(ramp(4, 5, 0), 2),
        platform(4, 6, "platform_2"), on(platform(4, 6, "platform_2"), 1)]}
    assert al.check(layout) == []

    # A ramp in the chain turned across it leaves the top stranded.
    layout["modules"][6] = on(ramp(6, 4, 0), 1)
    found = {}
    for p in al.check(layout):
        found.setdefault((p["col"], p["row"]), []).append(p["message"])

    assert any("side of a ramp" in m for m in found[(7, 4)])
    assert any("no robot can get onto" in m for m in found[(4, 6)])


# Up toward -x onto a one module landing 3" up, then turning up toward +y
# onto a 4" platform, as on the tower in the middle of test4.
LANDING = [
    ramp(7, 4, 90), platform(6, 4), on(ramp(6, 4, 90), 1),
    platform(5, 4, "platform_2"), on(ramp(5, 4, 90), 1),
    platform(4, 4, "platform_2"), on(platform(4, 4), 1),
    platform(4, 5, "platform_2"), on(platform(4, 5), 1),
    on(ramp(4, 5, 0), 2),
    platform(4, 6, "platform_2"), on(platform(4, 6, "platform_2"), 1)]


def test_chase_turns_up_a_ramp_chain_on_a_one_module_landing():
    from test_terrain import assert_never_climbs_an_edge, drive

    layout = {"size": 12, "modules": list(LANDING)}
    ground = al.terrain(layout)
    top = al.centre(layout, 4, 6)
    assert not ground.route(0.3, -0.15, *top, *top, math.pi).partial

    trail = drive(ground, 0.3, -0.15, math.pi, *top, steps=1500)
    assert_never_climbs_an_edge(ground, trail)
    # Turned on the 3" landing and up the ramp toward +y off it, short of
    # a target whose arm may be out.
    end_x, end_y = trail[-1]
    assert ground.height_at(end_x, end_y) > 0.0762 + 0.005
    assert end_x == pytest.approx(top[0], abs=0.01)


def test_a_hunter_under_a_target_on_a_platform_is_not_with_it():
    from robogame_sim.chase import Chaser

    layout = {"size": 12, "modules": list(LANDING)}
    ground = al.terrain(layout)
    x, y = al.centre(layout, 4, 6)
    chaser = Chaser(ground)
    chaser.track(x, y, 0.0)
    # On the floor beside the platform, 13 cm from the target.
    chaser.update(0.0, x - 0.13, y, 0.0)

    assert chaser.gap > 0.3
    assert chaser.status != "with the target"


# The walls round the tower in test4: two to a side, with an 8" gap in the
# middle of each side and a diagonal slot at each corner, which the robot
# only gets through lined up with it.
WALL_BOX = (
    [side("wall", c, 2, 180) for c in (3, 4, 7, 8)]
    + [side("wall", c, 9, 0) for c in (3, 4, 7, 8)]
    + [side("wall", 2, r, 90) for r in (3, 4, 7, 8)]
    + [side("wall", 9, r, 270) for r in (3, 4, 7, 8)])


def test_chase_squeezes_through_a_corner_slot():
    from robogame_sim import chase_sim
    from robogame_sim.chase import Chaser

    layout = {"size": 12, "modules": list(WALL_BOX)}
    world = chase_sim.load_world(layout)
    ground = world.terrain
    hunter = chase_sim.Robot(world, -0.30, -0.12, -math.pi / 2)
    tx, ty = -0.43, -0.43
    chaser = Chaser(ground)
    # With the target's arm up the hunter comes right into the slot.
    chaser.set_target_arm(math.pi / 2, math.pi / 2)
    command = (0.0, 0.0)
    slot = math.inf

    for step in range(int(10.0 / chase_sim.TICK)):
        chaser.track(tx, ty, step * chase_sim.TICK)
        command = chaser.update(step * chase_sim.TICK, hunter.x, hunter.y,
                                hunter.yaw)
        for _ in range(int(chase_sim.TICK / chase_sim.DT)):
            hunter.step(*command)
        slot = min(slot, math.dist((hunter.x, hunter.y), (-0.343, -0.343)))

    # It stops a centimetre short, its front through the slot's narrowest.
    assert chaser.gap < 0.02
    assert slot < 0.02, "into the slot, not round by the middle gap"
    # Lined up with it, without once pressing against a wall.
    assert abs(math.sin(hunter.yaw + 3 * math.pi / 4)) < math.sin(
        math.radians(10))
    assert hunter.blocked == 0


def test_a_join_between_ramps_side_by_side_is_ramp():
    # The meshes' float32 edges miss a cell centre right on the join.
    layout = {"size": 12, "modules": [ramp(c, 2, 0) for c in (2, 3, 4)]}
    ground = al.terrain(layout)
    x = al.centre(layout, 3, 2)[0] + al.MODULE / 2
    for y in (-0.38, -0.34):
        assert ground.height_at(x, y) == pytest.approx(
            ground.height_at(x - 0.02, y), abs=1e-3)


def test_a_wall_closes_the_side_it_stands_on():
    layout = {"size": 12, "modules": [
        ramp(5, 4, 0), platform(5, 5), on(side("wall", 5, 5, 0), 1),
        platform(5, 6)]}
    found = {(p["col"], p["row"]): p["message"] for p in al.check(layout)}

    assert "no robot can get onto" in found[(5, 6)]
    assert (5, 5) not in found


def test_chase_climbs_onto_a_second_storey():
    from test_terrain import assert_never_climbs_an_edge, drive

    layout = {"size": 12, "modules": list(TWO_STOREYS)}
    ground = al.terrain(layout)
    # Deep enough onto the 2" platform that the hunter does not stop short
    # on the ramp up to it.
    target = al.centre(layout, 5, 9)
    trail = drive(ground, 0.3, -0.45, math.pi, *target, steps=1500)
    assert_never_climbs_an_edge(ground, trail)
    assert ground.height_at(*trail[-1]) == pytest.approx(0.0508, abs=3e-3)


def test_a_wall_spans_two_floor_tiles():
    layout = {"size": 12, "modules": [side("wall", 5.5, 5, 0)]}
    ground = al.terrain(layout)
    x, y = al.centre(layout, 5.5, 5)
    y += al.PEG_LINE
    # Over the join between the tiles and the peg holes either side of it.
    for d in (-0.04, 0.0, 0.04):
        assert ground.height_at(x + d, y) == pytest.approx(0.0508, abs=1e-3)
    for d in (-0.06, 0.06):
        assert ground.height_at(x + d, y) == pytest.approx(0.0, abs=1e-3)

    text = al.world_sdf(layout, WORLD)
    assert 'collision name="wall_5p5_5"' in text


def test_walls_and_thwomps_share_a_tile_where_they_fit():
    fits = [
        # Side by side, either way round.
        [side("wall", 5, 5, 0), side("wall", 5, 5, 180)],
        [side("thwomp", 5, 5, 90), side("wall", 5, 5, 270)],
        # Staggered across two tiles.
        [side("wall", 4, 5, 0), side("wall", 4.5, 5, 180),
         side("wall", 5, 5, 0)],
        # One along each side of a ramp.
        [ramp(5, 5, 0), side("wall", 6, 5, 90), side("wall", 4, 5, 270)],
    ]
    for modules in fits:
        assert len(placed(al.normalise({"modules": modules}))) == \
            len(modules)

    for clash in (
            [side("wall", 5, 5, 0), side("wall", 5, 5, 90)],
            # The same peg line, from half a square up.
            [side("wall", 5, 5, 0), side("thwomp", 5, 5.5, 180)],
            [side("wall", 5, 5, 0), side("wall", 5.5, 5, 0)],
            [platform(5, 5), side("wall", 5.5, 5, 90)]):
        with pytest.raises(ValueError, match="overlaps"):
            al.normalise({"modules": clash})

    layout = {"size": 12, "modules": [side("wall", 5, 5, 0),
                                      side("wall", 5, 5, 180)]}
    ground = al.terrain(layout)
    x, y = al.centre(layout, 5, 5)
    for d in (-al.PEG_LINE, al.PEG_LINE):
        assert ground.height_at(x, y + d) == pytest.approx(0.0508, abs=1e-3)
    assert ground.height_at(x, y) == pytest.approx(0.0, abs=1e-3)

    text = al.world_sdf(layout, WORLD)
    assert 'collision name="wall_5_5"' in text
    assert 'collision name="wall_5_5_r180"' in text


def test_modules_go_in_steps_of_half_a_square():
    layout = al.normalise({"modules": [platform(5.5, 2.5), platform(1.0, 1)]})
    assert placed(layout) == [platform(1, 1), platform(5.5, 2.5)]
    assert isinstance(layout["modules"][0]["col"], int)
    assert '"col": 5.5, "row": 2.5' in al.dumps(layout)

    for bad in ({"modules": [platform(5.25, 2)]},
                {"modules": [platform(11.5, 2)]},
                {"modules": [side("wall", 0, 0, 90),
                             side("wall", -0.5, 0, 270)]},
                {"modules": [platform(5, 5), platform(5.5, 5.5)]}):
        with pytest.raises(ValueError):
            al.normalise(bad)

    # A wall's square can hang over the edge, as long as the wall is on
    # the grid.
    al.normalise({"modules": [side("wall", -0.5, 0, 270),
                              side("wall", 11.5, 3, 90)]})


def test_platforms_across_tiles_stand_where_they_are_put():
    layout = {"size": 12, "modules": [platform(5.5, 5.5)]}
    ground = al.terrain(layout)
    x, y = al.centre(layout, 5, 5)
    assert ground.height_at(x + 0.03, y + 0.03) == \
        pytest.approx(0.0254, abs=1e-3)
    assert ground.height_at(x - 0.03, y - 0.03) == \
        pytest.approx(0.0, abs=1e-3)


def test_modules_stack_across_platforms_of_one_height():
    al.normalise({"modules": [
        platform(5, 5), platform(6, 5), on(platform(5.5, 5), 1),
        on(side("wall", 6.5, 5, 90), 1)]})

    for bad in ([platform(5, 5), on(side("wall", 5.5, 5, 0), 1)],
                [platform(5, 5), platform(6, 5, "platform_2"),
                 on(side("wall", 5.5, 5, 0), 1)]):
        with pytest.raises(ValueError):
            al.normalise({"modules": bad})

    layout = {"size": 12, "modules": [
        platform(5, 5), platform(6, 5), on(platform(5.5, 5), 1)]}
    ground = al.terrain(layout)
    x, y = al.centre(layout, 5.5, 5)
    assert ground.height_at(x, y) == pytest.approx(0.0508, abs=1e-3)


def test_ramps_and_platforms_off_the_tiles_are_checked_the_same():
    # The ramps and platform of default.json, half a square along.
    layout = {"size": 12, "modules": [
        *[ramp(c + 0.5, 3, 0) for c in range(4, 8)],
        *[platform(c + 0.5, r) for c in range(4, 8) for r in range(4, 8)]]}
    assert al.check(layout) == []

    # A wall across the foot of two of the ramps, and a ramp that only
    # half meets the platform.
    blocked = layout["modules"] + [side("wall", 5, 2, 0),
                                   ramp(3.5, 7.5, 270)]
    found = {}
    for p in al.check({"size": 12, "modules": blocked}):
        found.setdefault((p["col"], p["row"]), []).append(p["message"])
    assert "runs into a wall" in found[(4.5, 3)][0]
    assert "runs into a wall" in found[(5.5, 3)][0]
    assert (6.5, 3) not in found
    assert "part of its high end" in found[(3.5, 7.5)][0]


def box_stl(sx, sy, sz):
    """An ASCII STL of a box sx x sy x sz, from 10, 20, 30 up."""
    lo = np.array([10.0, 20.0, 30.0])
    corners = [lo + np.array([i * sx, j * sy, k * sz])
               for i in (0, 1) for j in (0, 1) for k in (0, 1)]
    faces = []
    for axis in range(3):
        for side_ in (0, 1):
            quad = [c for n, c in enumerate(corners)
                    if (n >> (2 - axis)) & 1 == side_]
            faces += [(quad[0], quad[1], quad[3]), (quad[0], quad[3],
                                                    quad[2])]
    text = "solid box\n" + "".join(
        "facet normal 0 0 0\nouter loop\n" + "".join(
            f"vertex {x} {y} {z}\n" for x, y, z in face)
        + "endloop\nendfacet\n" for face in faces) + "endsolid box\n"
    return text.encode()


@pytest.fixture
def parts(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBOGAME_LAYOUTS", str(tmp_path))
    yield tmp_path
    monkeypatch.delenv("ROBOGAME_LAYOUTS")
    al.refresh_parts()


def test_dropped_stl_is_scaled_and_centred(parts):
    # 3 x 1.5 x 2 inches, in millimetres, guessed from its size.
    part = al.add_part("bumper", box_stl(76.2, 38.1, 50.8))
    assert part["units"] == "mm"
    assert np.allclose(part["size"], [0.0762, 0.0381, 0.0508])

    points = read_stl(part["path"]).reshape(-1, 3)
    assert np.allclose(points.min(axis=0), [-0.0381, -0.01905, 0])
    assert np.allclose(points.max(axis=0), [0.0381, 0.01905, 0.0508])
    assert os.path.dirname(part["path"]) == str(parts / "parts")
    # Rounded up to 2 inch cells, and turned with the part.
    assert al.block_cells("bumper", 0) == (2, 1)
    assert al.block_cells("bumper", 90) == (1, 2)
    assert al.modules_info()["types"]["bumper"]["cells"] == [2, 1]


def test_dropped_stl_with_y_up_stands_on_z(parts):
    part = al.add_part("tall", box_stl(1, 4, 2), units="inch", up="y")
    # 4 inches along y is its height now.
    assert np.allclose(part["size"], [0.0254, 0.0508, 0.1016])


def test_bad_stls_are_refused(parts):
    with pytest.raises(ValueError):
        al.add_part("ramp", box_stl(1, 1, 1), units="inch")
    with pytest.raises(ValueError):
        al.add_part("junk", b"not an stl at all")
    with pytest.raises(ValueError):
        # 2 m across in inches.
        al.add_part("huge", box_stl(80, 80, 1), units="inch")
    with pytest.raises(ValueError):
        al.add_part("../escape", box_stl(1, 1, 1))


def test_dropped_part_is_a_block_in_layouts_and_the_world(parts):
    al.add_part("crate", box_stl(3, 3, 2), units="inch")
    layout = al.normalise({"modules": [
        {"type": "crate", "col": 5, "row": 5, "rotation": 0},
        ramp(5, 4, 0)]})
    crate = placed(layout)[1]
    # 2 x 2 cells around the square, and half a cell on: 3 inches is
    # 1.5 cells, so 2, centred on the square.
    assert sorted(al.cells(crate)) == [(10, 10), (10, 11), (11, 10),
                                       (11, 11)]
    with pytest.raises(ValueError):
        al.normalise({"modules": [
            {"type": "crate", "col": 5, "row": 5},
            platform(5, 5)]})

    # The ramp runs into it.
    problems = al.check(layout)
    assert any(p["type"] == "ramp" and "crate" in p["message"]
               for p in problems)

    sdf = al.model_sdf(layout)
    assert f"file://{parts}/parts/crate.stl" in sdf
    ground = al.terrain(layout)
    x, y = al.centre(layout, 5, 5)
    assert ground.height_at(x, y) == pytest.approx(0.0508, abs=0.005)


def test_odd_blocks_sit_half_a_cell_off_the_square(parts):
    al.add_part("stick", box_stl(1, 4, 1), units="inch")
    m = {"type": "stick", "col": 2, "row": 2, "rotation": 0}
    assert sorted(al.cells(m)) == [(5, 4), (5, 5)]
    assert al.block_offset("stick", 0) == (pytest.approx(0.0254), 0)
    m["rotation"] = 90
    assert sorted(al.cells(m)) == [(4, 5), (5, 5)]


def test_editor_takes_a_dropped_stl(editor):
    url, folder = editor
    request = urllib.request.Request(url + "/api/parts/block_1?units=inch",
                                     data=box_stl(2, 2, 2), method="PUT")
    with urllib.request.urlopen(request) as response:
        body = json.loads(response.read())
    assert body["part"]["type"] == "block_1"
    assert body["modules"]["types"]["block_1"]["mount"] == "block"
    assert (folder / "parts" / "block_1.stl").is_file()
    # The 3D view fetches its mesh.
    mesh = body["modules"]["types"]["block_1"]["mesh"]
    status, data = call(url + "/api/meshes/" + mesh)
    assert status == 200 and len(read_stl_bytes(data)) == 12

    status, _ = call(url + "/api/parts/platform", "PUT", {})
    assert status == 400
    al.refresh_parts()


@pytest.mark.parametrize("kind", ["ramp", "wall", "thwomp", "platform_2"])
@pytest.mark.parametrize("rotation", al.ROTATIONS)
def test_editor_3d_poses_are_the_sims(kind, rotation):
    # The 3D view adds a square's centre and the platforms' height to the
    # pose at 0, 0, which must give what the sim's world uses.
    part = al.PARTS[kind]
    x, y, z, yaw = al.modules_info()["types"][kind]["poses"][
        rotation]
    (px, py, pz, pyaw), _ = al.placement(part, 0.3, -0.2,
                                                   rotation, 0.0254)
    assert (x + 0.3, y - 0.2, z + 0.0254, yaw) == pytest.approx(
        (px, py, pz, pyaw), abs=1e-5)


def test_parts_are_renamed_deleted_to_the_archive_and_restored(parts):
    al.add_part("crate", box_stl(3, 3, 2), units="inch")
    al.save("yard", {"modules": [{"type": "crate", "col": 5, "row": 5,
                                  "rotation": 0}]})

    # Renamed, in the layout too.
    assert al.rename_part("crate", "box") == ("box", ["yard"])
    assert "box" in al.PARTS and "crate" not in al.PARTS
    assert placed(al.load("yard"))[0]["type"] == "box"
    with pytest.raises(ValueError):
        al.rename_part("ramp", "slope")
    with pytest.raises(ValueError):
        al.rename_part("box", "wall")

    # Deleted, twice under one name, and kept in the archive.
    assert al.archive_part("box") == "box"
    assert "box" not in al.PARTS
    with pytest.raises(ValueError):
        al.load("yard")
    al.add_part("box", box_stl(1, 1, 1), units="inch")
    assert al.archive_part("box") == "box.2"
    archive = al.archived_parts()
    assert sorted(p["name"] for p in archive) == ["box", "box.2"]
    assert {p["part"] for p in archive} == {"box"}
    with pytest.raises(ValueError):
        al.archive_part("wall")

    # Restored under its own name, and the layout opens again.
    assert al.restore_part("box") == "box"
    assert placed(al.load("yard"))[0]["type"] == "box"
    with pytest.raises(ValueError):
        al.restore_part("box.2")
    assert al.restore_part("box.2", "small_box") == "small_box"
    assert al.archived_parts() == []
    with pytest.raises(ValueError):
        al.restore_part("../escape")


def test_part_names_have_spaces_and_files_dashes(parts):
    part = al.add_part("big  crate ", box_stl(3, 3, 2), units="inch")
    assert part["type"] == "big-crate" and part["label"] == "big crate"
    assert os.path.basename(part["path"]) == "big-crate.stl"
    assert al.modules_info()["types"]["big-crate"]["label"] == "big crate"
    with pytest.raises(ValueError):
        al.add_part("big/crate", box_stl(1, 1, 1), units="inch")

    assert al.rename_part("big-crate", "small box") == ("small-box", [])
    assert al.PARTS["small-box"].label == "small box"
    # Only the name's case or spacing changed: the same file.
    assert al.rename_part("small-box", "small  box")[0] == "small-box"
    al.archive_part("small-box")
    assert al.archived_parts()[0]["label"] == "small box"
    assert al.restore_part("small-box") == "small-box"
    assert al.PARTS["small-box"].label == "small box"


def test_rename_leaves_no_installed_copy_behind(parts, tmp_path,
                                               monkeypatch):
    # colcon installs a copy of each part, or a link to it, in share.
    share = tmp_path / "share"
    (share / "layouts" / "parts").mkdir(parents=True)
    monkeypatch.setattr(al, "share_dir", lambda: str(share))
    path = al.add_part("crate", box_stl(3, 3, 2), units="inch")["path"]
    copy = share / "layouts" / "parts" / "crate.stl"
    copy.write_bytes(open(path, "rb").read())

    al.rename_part("crate", "box")
    assert "crate" not in al.PARTS and "box" in al.PARTS
    assert not copy.exists()

    # A link left behind is not a part.
    os.symlink(str(parts / "parts" / "gone.stl"), str(copy))
    al.refresh_parts()
    assert "crate" not in al.PARTS


def test_editor_routes_for_parts(parts):
    from robogame_sim import arena_editor
    al.add_part("crate", box_stl(3, 3, 2), units="inch")
    status, body = arena_editor.post("/api/parts/crate/rename",
                                     lambda: {"name": "box"})
    assert body["name"] == "box"
    assert status == 200 and "box" in body["modules"]["types"]
    status, body = arena_editor.post("/api/parts/box/delete", lambda: {})
    assert status == 200 and body["archived"] == "box"
    assert arena_editor.get("/api/archive")[1]["archive"][0]["part"] == "box"
    status, body = arena_editor.post("/api/archive/box/restore", lambda: {})
    assert status == 200 and body["name"] == "box" and body["archive"] == []
    assert arena_editor.post("/api/parts/ramp/delete", dict)[0] == 400


def bare(*modules):
    return {"size": 12, "base": "quadrants", "modules": list(modules)}


def tile(col, row):
    return {"type": "floor", "col": col, "row": row}


def test_a_new_layout_is_bare_quadrant_plates():
    layout = al.normalise(bare())
    assert layout["modules"] == [] and layout["base"] == "quadrants"
    sdf = al.model_sdf(layout)
    assert sdf.count("<collision") == 4 and "floor_" not in sdf
    ground = al.terrain(layout)
    # The plates' top is a tile's thickness under z = 0.
    assert ground.height_at(0.1, 0.1) == pytest.approx(-0.0127, abs=1e-4)


def test_old_layouts_get_a_tile_on_every_square():
    layout = al.normalise({"modules": [platform(5, 5)]})
    assert len(layout["modules"]) == 145
    assert al.dumps(layout).count('"floor"') == 144
    # Saved and loaded again, it is the same.
    assert al.normalise(json.loads(al.dumps(layout))) == layout


def test_modules_on_the_bare_plate_stand_a_tile_lower():
    ground = al.terrain(bare(tile(2, 2), platform(2, 2), platform(8, 8)))
    on_tile = ground.height_at(*al.centre({"size": 12}, 2, 2))
    on_plate = ground.height_at(*al.centre({"size": 12}, 8, 8))
    assert on_tile == pytest.approx(0.0254, abs=1e-4)
    assert on_plate == pytest.approx(0.0254 - 0.0127, abs=1e-4)


def test_floor_tiles_go_on_whole_squares_under_everything():
    for wrong in ([tile(2.5, 2)], [dict(tile(2, 2), level=1)],
                  # Half on a tile and half on the plate.
                  [tile(2, 2), platform(2.5, 2)]):
        with pytest.raises(ValueError):
            al.normalise(bare(*wrong))


def test_checks_go_by_the_floor_under_a_platform():
    # A ramp on the plate up to a 1" platform on the plate is fine, and
    # one from the plate to a platform on a tile is half an inch short.
    assert al.check(bare(platform(5, 5), ramp(5, 4, 0))) == []
    problems = al.check(bare(tile(5, 5), platform(5, 5), ramp(5, 4, 0)))
    assert [p["message"] for p in problems if p["type"] == "ramp"] == [
        'its high end meets a 1" platform, half an inch above the top of '
        'the ramp']
    # A tile's edge is driven over, so tiles here and there are no island.
    assert al.check(bare(tile(5, 5), tile(5, 4), tile(5, 3), platform(5, 5),
                         ramp(5, 4, 0))) == []


def test_a_module_keeps_its_own_colour_into_the_sim():
    layout = al.normalise(bare(dict(platform(5, 5), colour="#E0A030 "),
                               platform(2, 2)))
    painted = [m for m in layout["modules"] if "colour" in m]
    assert [m["colour"] for m in painted] == ["#e0a030"]
    assert json.loads(al.dumps(layout)) == layout
    sdf = al.model_sdf(layout)
    assert "<diffuse>0.878 0.627 0.188 1</diffuse>" in sdf
    r, g, b = al.PARTS["platform"].colour
    assert f"<diffuse>{r} {g} {b} 1</diffuse>" in sdf
    with pytest.raises(ValueError):
        al.normalise(bare(dict(platform(5, 5), colour="red")))
