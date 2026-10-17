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
    assert layout["modules"] == [
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
        assert len(al.normalise({"modules": modules})["modules"]) == \
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
    assert layout["modules"] == [platform(1, 1), platform(5.5, 2.5)]
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
