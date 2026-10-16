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
from robogame_sim.terrain import Terrain

HERE = os.path.dirname(__file__)
PACKAGE = os.path.join(HERE, "..")
CAD = os.path.join(PACKAGE, "models", al.CAD_MODEL, "model.sdf")
WORLD = os.path.join(PACKAGE, "worlds", "robogame_arena.sdf")


def platform(col, row):
    return {"type": "platform", "col": col, "row": row}


def ramp(col, row, rotation):
    return {"type": "ramp", "col": col, "row": row, "rotation": rotation}


def test_default_layout_is_the_cad_arena():
    layout = al.load(os.path.join(PACKAGE, "layouts", "default.json"))
    built = al.terrain(layout)
    cad = Terrain.from_sdf(CAD)

    assert built.shape == cad.shape
    assert np.allclose(np.nan_to_num(built.height, nan=-1),
                       np.nan_to_num(cad.height, nan=-1))
    assert (built.free == cad.free).all()
    assert al.check(layout) == []


@pytest.mark.parametrize("rotation", al.ROTATIONS)
def test_ramps_rise_the_way_they_are_turned(rotation):
    ground = al.terrain({"size": 9, "modules": [ramp(4, 4, rotation)]})
    ux, uy = al.uphill(rotation)
    low = ground.height_at(-0.05 * ux, -0.05 * uy)
    high = ground.height_at(0.05 * ux, 0.05 * uy)
    assert high - low > 0.01


def test_turned_ramp_leads_onto_its_platform():
    # Platform west of the ramp, so the ramp rises toward -x.
    layout = {"size": 9, "modules": [
        platform(2, 4), platform(2, 5), platform(1, 4), platform(1, 5),
        ramp(3, 4, 90), ramp(3, 5, 90)]}
    assert al.check(layout) == []
    ground = al.terrain(layout)
    x, y = al.centre(layout, 3, 4)
    cell = ground.cell(x, y)
    ax, ay = ground.axis[cell]
    assert abs(ax) > 0.99 and abs(ay) < 0.01


def test_malformed_layouts_are_refused():
    for bad in ({"modules": [platform(9, 0)]},
                {"modules": [platform(1, 1), ramp(1, 1, 0)]},
                {"modules": [ramp(1, 1, 45)]},
                {"modules": [{"type": "wall", "col": 1, "row": 1}]}):
        with pytest.raises(ValueError):
            al.normalise(bad)


def test_checks_find_ramps_and_platforms_robots_cannot_use():
    layout = {"size": 9, "modules": [
        platform(1, 7), platform(1, 6), platform(2, 7), platform(2, 6),
        ramp(0, 7, 270),               # low end against the arena edge
        ramp(6, 2, 0),                 # leads to nothing
        platform(6, 6),                # a lone module, too narrow
        ramp(4, 4, 0), platform(4, 3)  # low end runs into a platform
    ]}
    found = {}
    for p in al.check(layout):
        found[(p["col"], p["row"])] = found.get((p["col"], p["row"]), "") \
            + p["message"]

    assert "edge of the arena" in found[(0, 7)]
    assert "no platform" in found[(6, 2)]
    assert "too narrow" in found[(6, 6)]
    assert "runs into a platform" in found[(4, 4)]
    assert "no robot can get onto" in found[(1, 7)]


def test_world_has_the_layout_instead_of_the_cad_include():
    layout = {"size": 9, "modules": [platform(4, 1), platform(4, 2)]}
    text = al.world_sdf(layout, WORLD)

    assert "<uri>model://robogame_arena_cad</uri>" not in text
    assert 'collision name="platform_4_1"' in text
    assert text.count('<collision name="floor_') == 81
    # token_4 sits at (0, -0.3), now on the platform at col 4, row 1.
    z = float(text.split('<model name="token_4">')[1]
              .split("<pose>")[1].split()[2])
    assert z > 0.0254


def test_layouts_save_and_load(tmp_path):
    layout = {"size": 9, "modules": [ramp(3, 2, 180), platform(3, 1)]}
    path = al.save("trial", layout, folder=str(tmp_path))
    assert al.load(path) == al.normalise(layout)
    saved = json.loads(open(path).read())["modules"]
    assert [m.get("rotation") for m in saved if m["type"] == "ramp"] == [180]
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

    layout = {"size": 9, "modules": [ramp(4, 3, 0), platform(4, 4)]}
    status, body = call(url + "/api/layouts/mine", "PUT", layout)
    assert status == 200
    assert os.path.exists(folder / "mine.json")
    assert json.loads(body)["problems"] == []

    status, body = call(url + "/api/layouts")
    assert "mine" in json.loads(body)["layouts"]

    status, body = call(url + "/api/layouts/mine")
    assert json.loads(body) == al.normalise(layout)

    status, body = call(url + "/api/check", "POST",
                        {"size": 9, "modules": [ramp(0, 0, 180)]})
    assert status == 200 and json.loads(body)["problems"]

    assert call(url + "/api/layouts/bad%20name", "PUT", layout)[0] == 400
    assert call(url + "/api/layouts/nothere")[0] == 404
    assert call(url + "/api/check", "POST",
                {"modules": [ramp(1, 1, 45)]})[0] == 400


def test_chase_climbs_a_turned_ramp():
    from test_terrain import assert_never_climbs_an_edge, drive

    layout = {"size": 9, "modules": [
        platform(c, r) for c in (0, 1, 2) for r in (5, 6, 7)]
        + [ramp(3, r, 90) for r in (5, 6, 7)]}
    ground = al.terrain(layout)
    # Far enough onto the platform that the hunter cannot reach it from
    # the floor in front of the ramp.
    target = al.centre(layout, 0, 6)
    trail = drive(ground, 0.3, -0.3, math.pi / 2, *target)
    assert_never_climbs_an_edge(ground, trail)
    assert ground.height_at(*trail[-1]) > 0.02


def test_unreachable_target_is_waited_for_beside_its_platform():
    from test_terrain import assert_never_climbs_an_edge, drive

    layout = {"size": 9, "modules": [platform(6, 6), platform(6, 5)]}
    ground = al.terrain(layout)
    target = al.centre(layout, 6, 6)

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
