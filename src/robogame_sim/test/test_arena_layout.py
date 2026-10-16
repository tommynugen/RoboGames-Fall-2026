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
from robogame_sim.arena_editor import Handler
from robogame_sim.terrain import Terrain, read_stl

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
    assert layout["modules"] == [platform(1, 1, "platform_2"),
                                 side("thwomp", 2, 1, 90)]


def test_checks_find_ramps_and_platforms_robots_cannot_use():
    layout = {"size": 12, "modules": [
        platform(1, 7), platform(1, 6), platform(2, 7), platform(2, 6),
        ramp(0, 7, 270),                # low end against the arena edge
        ramp(6, 2, 0),                  # leads to nothing
        platform(9, 9),                 # a lone module, too narrow
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
    assert "too narrow" in found[(9, 9)]
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
    saved = json.loads(open(path).read())["modules"]
    assert [m.get("rotation") for m in saved] == [None, 180, 270]
    with pytest.raises(ValueError):
        al.save("../escape", layout, folder=str(tmp_path))


@pytest.fixture
def editor(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBOGAME_LAYOUTS", str(tmp_path))
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
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
    assert status == 200 and b"Arena layout" in page

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

    assert call(url + "/api/layouts/bad%20name", "PUT", layout)[0] == 400
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
