"""Arena layouts: which module sits on each square of the arena floor.

The arena is a square grid of 0.127 m modules, 9 x 9, centred on the
origin. Every square has a floor tile. A layout lists what stands on them:

    {
      "size": 9,
      "modules": [
        {"type": "platform", "col": 4, "row": 4},
        {"type": "ramp", "col": 4, "row": 3, "rotation": 0}
      ]
    }

col counts along +x and row along +y, both from 0 at the -x, -y corner. A
ramp's rotation is in degrees anticlockwise seen from above: at 0 it rises
toward +y, at 90 toward -x, at 180 toward -y and at 270 toward +x.

From a layout this builds the Gazebo model of the arena, out of the meshes
scripts/export_onshape_arena.py exported from the CAD, a world around it,
and the terrain chase_node plans over, so the sim and the planner always
see the same arena. Layouts are made with arena_editor.
"""

import json
import math
import os
import re
import tempfile
import xml.etree.ElementTree as ElementTree

import numpy as np

from robogame_sim.terrain import Terrain, sdf_triangles

MODULE = 0.127
SIZE = 9
TYPES = ("platform", "ramp")
ROTATIONS = (0, 90, 180, 270)
CAD_MODEL = "robogame_arena_cad"

# Each module as the CAD export places it, relative to the middle of its
# square: (mesh, mesh z, collision box size and z, or None to collide with
# the mesh, colour).
FLOOR = ("floor_module.stl", -0.0064, ((MODULE, MODULE, 0.0127), -0.0064),
         (0.647, 0.647, 0.647))
PARTS = {
    "platform": ("platform_module_1.stl", -0.0064,
                 ((MODULE, MODULE, 0.0254), 0.01265),
                 (0.231, 0.380, 0.706)),
    "ramp": ("ramp_module_curved.stl", -0.0826, None,
             (0.616, 0.812, 0.929)),
}

NAME = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
HERE = os.path.dirname(os.path.realpath(__file__))


# -- where things are ------------------------------------------------------

def share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("robogame_sim")
    except Exception:
        return None


def source_dir():
    """The robogame_sim source folder, if this runs from a workspace."""
    candidates = [os.path.dirname(HERE)]
    share = share_dir()

    if share:
        # <ws>/install/robogame_sim/share/robogame_sim
        workspace = os.path.normpath(os.path.join(share, *[".."] * 4))
        candidates.append(os.path.join(workspace, "src", "robogame_sim"))

    for candidate in candidates:
        if os.path.exists(os.path.join(candidate, "package.xml")):
            return candidate

    return None


def layout_dirs():
    """Folders layouts are looked for in, the one saved to first."""
    dirs = []

    if os.environ.get("ROBOGAME_LAYOUTS"):
        dirs.append(os.environ["ROBOGAME_LAYOUTS"])

    source = source_dir()
    if source:
        dirs.append(os.path.join(source, "layouts"))
    else:
        dirs.append(os.path.expanduser("~/.local/share/robogame_sim/layouts"))

    share = share_dir()
    if share:
        dirs.append(os.path.join(share, "layouts"))

    unique = []
    for folder in dirs:
        if folder not in unique:
            unique.append(folder)

    return unique


def mesh_dir():
    for root in (share_dir(), source_dir()):
        if root:
            meshes = os.path.join(root, "models", CAD_MODEL, "meshes")
            if os.path.isdir(meshes):
                return meshes

    raise FileNotFoundError(
        f"no {CAD_MODEL} meshes, run scripts/export_onshape_arena.py")


def list_layouts():
    names = set()

    for folder in layout_dirs():
        if os.path.isdir(folder):
            names.update(name[:-5] for name in os.listdir(folder)
                         if name.endswith(".json"))

    return sorted(names)


def find_layout(name):
    """Path of a layout given by name or path."""
    if os.path.isfile(name):
        return os.path.abspath(name)

    for folder in layout_dirs():
        path = os.path.join(folder, f"{name}.json")
        if os.path.isfile(path):
            return path

    raise FileNotFoundError(
        f"no layout {name!r} in {', '.join(layout_dirs())}")


# -- reading and writing ---------------------------------------------------

def normalise(layout):
    """A checked copy of a layout, raising ValueError if it is malformed."""
    size = int(layout.get("size", SIZE))
    if not 3 <= size <= 25:
        raise ValueError(f"size must be 3 to 25, not {size}")

    modules = []
    taken = set()

    for module in layout.get("modules", []):
        kind = module.get("type")
        col, row = int(module.get("col")), int(module.get("row"))

        if kind not in TYPES:
            raise ValueError(f"unknown module type {kind!r}")
        if not (0 <= col < size and 0 <= row < size):
            raise ValueError(f"{kind} at col {col}, row {row} is off the grid")
        if (col, row) in taken:
            raise ValueError(f"two modules at col {col}, row {row}")
        taken.add((col, row))

        entry = {"type": kind, "col": col, "row": row}
        if kind == "ramp":
            rotation = int(module.get("rotation", 0)) % 360
            if rotation not in ROTATIONS:
                raise ValueError(f"ramp rotation must be one of {ROTATIONS}")
            entry["rotation"] = rotation
        modules.append(entry)

    modules.sort(key=lambda m: (m["row"], m["col"]))
    return {"size": size, "modules": modules}


def load(name):
    with open(find_layout(name)) as file:
        return normalise(json.load(file))


def save(name, layout, folder=None):
    if not NAME.match(name):
        raise ValueError("names are letters, digits, - and _ only")

    folder = folder or layout_dirs()[0]
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"{name}.json")
    layout = normalise(layout)

    with open(path, "w") as file:
        file.write(dumps(layout))

    return path


def dumps(layout):
    """JSON with one module per line, so layouts diff well."""
    rows = ",\n".join("    " + json.dumps(m) for m in layout["modules"])
    body = f"[\n{rows}\n  ]" if rows else "[]"
    return f'{{\n  "size": {layout["size"]},\n  "modules": {body}\n}}\n'


# -- geometry --------------------------------------------------------------

def centre(layout, col, row):
    half = (layout["size"] - 1) / 2
    return (col - half) * MODULE, (row - half) * MODULE


def uphill(rotation):
    """Unit vector a ramp at this rotation rises along."""
    angle = math.radians(rotation)
    return -math.sin(angle), math.cos(angle)


def model_sdf(layout, name="arena"):
    """The arena as an SDF <model>, meshes as model://robogame_arena_cad."""
    parts = []

    def add(label, part, x, y, yaw):
        mesh, mesh_z, box, (r, g, b) = part
        uri = f"model://{CAD_MODEL}/meshes/{mesh}"
        pose = f"{x:.5f} {y:.5f} {mesh_z:.5f} 0 0 {yaw:.5f}"
        material = (f"<material><ambient>{r} {g} {b} 1</ambient>"
                    f"<diffuse>{r} {g} {b} 1</diffuse>"
                    f"<specular>0.1 0.1 0.1 1</specular></material>")

        if box:
            (sx, sy, sz), box_z = box
            collision = (f"<pose>{x:.5f} {y:.5f} {box_z:.5f} 0 0 {yaw:.5f}"
                         f"</pose><geometry><box><size>{sx} {sy} {sz}</size>"
                         f"</box></geometry>")
        else:
            collision = (f"<pose>{pose}</pose><geometry><mesh><uri>{uri}"
                         f"</uri></mesh></geometry>")

        parts.append(
            f'      <visual name="{label}_visual"><pose>{pose}</pose>'
            f"<geometry><mesh><uri>{uri}</uri></mesh></geometry>{material}"
            f"</visual>\n"
            f'      <collision name="{label}">{collision}</collision>')

    size = layout["size"]
    for row in range(size):
        for col in range(size):
            add(f"floor_{col}_{row}", FLOOR, *centre(layout, col, row), 0.0)

    for m in layout["modules"]:
        yaw = math.radians(m.get("rotation", 0))
        add(f"{m['type']}_{m['col']}_{m['row']}", PARTS[m["type"]],
            *centre(layout, m["col"], m["row"]), yaw)

    return (f'<model name="{name}">\n    <static>true</static>\n'
            f'    <link name="link">\n' + "\n".join(parts)
            + "\n    </link>\n  </model>")


def terrain(layout):
    """The terrain chase_node plans over, for this layout."""
    meshes = mesh_dir()

    def resolve(uri):
        return os.path.join(meshes, uri.rsplit("/", 1)[-1])

    root = ElementTree.fromstring(model_sdf(layout))
    return Terrain(sdf_triangles(root, resolve))


INCLUDE = re.compile(
    r"<include>\s*<uri>model://" + CAD_MODEL + r"</uri>\s*</include>")
TOKEN_POSE = re.compile(
    r'(<model name="token_\d+">\s*<pose>)(\S+) (\S+) (\S+)( [^<]*</pose>)')


def world_sdf(layout, base_world, ground=None):
    """The base world with its CAD arena swapped for this layout. Tokens
    are lifted onto whatever they would otherwise be inside."""
    with open(base_world) as file:
        text = file.read()

    if not INCLUDE.search(text):
        raise ValueError(f"{base_world} has no model://{CAD_MODEL} include")

    text = INCLUDE.sub(lambda _: model_sdf(layout), text)
    ground = ground or terrain(layout)

    def lift(match):
        x, y, z = (float(v) for v in match.group(2, 3, 4))
        under = ground.height_at(x, y)
        if not math.isnan(under):
            z = max(z, under + 0.01)
        return f"{match.group(1)}{x} {y} {z:.4f}{match.group(5)}"

    return TOKEN_POSE.sub(lift, text)


def write_world(layout, base_world, ground=None):
    folder = tempfile.mkdtemp(prefix="robogame_world_")
    path = os.path.join(folder, "robogame_arena.sdf")

    with open(path, "w") as file:
        file.write(world_sdf(layout, base_world, ground))

    return path


# -- checks ----------------------------------------------------------------

def check(layout, ground=None):
    """Problems with a layout that a robot would run into, as dicts with
    col, row and message."""
    layout = normalise(layout)
    size = layout["size"]
    at = {(m["col"], m["row"]): m for m in layout["modules"]}
    problems = []

    for m in layout["modules"]:
        if m["type"] != "ramp":
            continue

        dx, dy = (round(v) for v in uphill(m["rotation"]))
        low = (m["col"] - dx, m["row"] - dy)
        high = (m["col"] + dx, m["row"] + dy)

        if not (0 <= low[0] < size and 0 <= low[1] < size):
            problems.append(dict(m, message="its low end is at the edge of "
                                 "the arena, so it cannot be driven onto"))
        elif low in at:
            problems.append(dict(m, message=f"its low end runs into a "
                                 f"{at[low]['type']}"))

        if at.get(high, {}).get("type") != "platform":
            problems.append(dict(m, message="its high end leads to no "
                                 "platform"))

    ground = ground or terrain(layout)
    floor = main_floor(ground)

    if floor is None:
        return problems

    for m in layout["modules"]:
        if m["type"] != "platform":
            continue

        x, y = centre(layout, m["col"], m["row"])
        cells = module_cells(ground, x, y)

        if not cells:
            problems.append(dict(m, message="too narrow for a robot to "
                                 "drive on, so it is only an obstacle"))
        elif not any(floor[c] for c in cells):
            problems.append(dict(m, message="no robot can get onto this "
                                 "platform from the floor"))

    return problems


def module_cells(ground, x, y):
    """Open cells under a module's square."""
    cells = []
    step = ground.resolution

    for ox in np.arange(-MODULE / 2 + step / 2, MODULE / 2, step):
        for oy in np.arange(-MODULE / 2 + step / 2, MODULE / 2, step):
            cell = ground.cell(x + ox, y + oy)
            if ground.inside(cell) and ground.free[cell]:
                cells.append(cell)

    return cells


def main_floor(ground):
    """The largest stretch of ground robots on the floor can drive to."""
    low = ground.free & (np.nan_to_num(ground.height, nan=1.0) < 0.005)
    best, best_count = None, 0
    seen = np.zeros(ground.shape, dtype=bool)

    for i, j in zip(*np.nonzero(low)):
        if seen[i, j]:
            continue

        x, y = ground.centre((i, j))
        region = ground.reachable(x, y)
        seen |= region & low
        count = int(region.sum())

        if count > best_count:
            best, best_count = region, count

    return best
