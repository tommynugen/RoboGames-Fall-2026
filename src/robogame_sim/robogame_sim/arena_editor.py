"""Browser editor for arena layouts.

    ros2 run robogame_sim robogame_app

The Arena editor tab of the RoboGame app (robogame_app.py), at
http://localhost:8766/#editor. This file has the editor's page API, which
the app serves; arena_editor.html is the page. `ros2 run robogame_sim
arena_editor` opens the app on this tab.

A page where the arena's modules, 1 and 2
inch platforms, ramps, walls and the thwomp, are placed on the floor grid
as they are set up on the table, and turned to face the way they do. Layouts are saved as JSON into the
workspace's src/robogame_sim/layouts, where sim.launch.py layout:=NAME and
chase_node find them, and checked as they are edited for ramps that cannot
be driven and platforms no robot can reach. An STL file dragged onto the
page is saved as a new module type, in layouts/parts.
"""

import os
from urllib.parse import parse_qs, unquote, urlsplit

from robogame_sim import arena_layout

HERE = os.path.dirname(os.path.realpath(__file__))
PAGE = os.path.join(HERE, "arena_editor.html")
PREFIX = "/api/layouts/"
# The editor's 3D view.
VIEW3D = os.path.join(HERE, "arena_view3d.js")
MESHES = "/api/meshes/"
PARTS = "/api/parts/"


def layout_name(path):
    name = unquote(path[len(PREFIX):])
    if not arena_layout.NAME.match(name):
        raise ValueError("names are letters, digits, - and _ only")
    return name


def get(path):
    """The editor's GET routes: (status, body), or None for a path that is
    not the editor's."""
    try:
        if path == "/api/modules":
            return 200, arena_layout.modules_info()
        if path == "/api/layouts":
            return 200, {"layouts": arena_layout.list_layouts(),
                         "folder": arena_layout.layout_dirs()[0]}
        if path.startswith(PREFIX):
            return 200, arena_layout.load(layout_name(path))
        if path.startswith(MESHES):
            # Only the arena's own meshes, by file name, for the 3D view,
            # and the STL parts as parts/NAME.stl.
            name = path[len(MESHES):]
            part = arena_layout.PARTS.get(name[len("parts/"):-len(".stl")])
            if (name.startswith("parts/") and part is not None
                    and part.mount == "block"):
                with open(part.mesh, "rb") as mesh:
                    return 200, mesh.read(), "model/stl"
            meshes = {p.mesh for p in arena_layout.PARTS.values()}
            if name not in meshes | {arena_layout.FLOOR.mesh}:
                return 404, {"error": "no such mesh"}
            with open(os.path.join(arena_layout.mesh_dir(), name),
                      "rb") as mesh:
                return 200, mesh.read(), "model/stl"
    except FileNotFoundError as error:
        return 404, {"error": str(error)}
    except ValueError as error:
        return 400, {"error": str(error)}
    return None


def put(path, read_body, read_raw=None):
    """Saves a layout, or an STL part. read_body gives the request's JSON,
    and read_raw(limit) its bytes, refusing more than limit."""
    if path.startswith(PARTS) and read_raw is not None:
        return put_part(path, read_raw)
    if not path.startswith(PREFIX):
        return None
    try:
        layout = arena_layout.normalise(read_body())
        saved = arena_layout.save(layout_name(path), layout)
        return 200, {"path": saved, "problems": arena_layout.check(layout)}
    except (ValueError, TypeError, KeyError) as error:
        return 400, {"error": str(error)}


def put_part(path, read_raw):
    """An STL dropped onto the page, as the body, saved as a module type
    named by the path. ?units= is auto, m, cm, mm or inch and ?up= is z or
    y, the axis that points up in the file."""
    try:
        url = urlsplit(path)
        query = {k: v[-1] for k, v in parse_qs(url.query).items()}
        part = arena_layout.add_part(
            unquote(url.path[len(PARTS):]), read_raw(arena_layout.MAX_STL),
            query.get("units", "auto"), query.get("up", "z"))
        part.pop("path")
        return 200, {"part": part, "modules": arena_layout.modules_info()}
    except (ValueError, TypeError) as error:
        return 400, {"error": str(error)}


def post(path, read_body):
    """Checks a layout without saving it."""
    if path != "/api/check":
        return None
    try:
        return 200, {"problems": arena_layout.check(read_body())}
    except (ValueError, TypeError, KeyError) as error:
        return 400, {"error": str(error)}


def main(args=None):
    from robogame_sim import robogame_app
    robogame_app.main(args, tab="editor")


if __name__ == "__main__":
    main()
