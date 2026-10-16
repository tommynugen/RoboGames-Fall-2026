"""Arena layouts: which module sits on each square of the arena floor.

The arena is a square grid of 4 inch (0.1016 m) modules, 12 x 12, centred
on the origin. Every square has a floor tile. A layout lists what stands on
them, one module to a square:

    {
      "size": 12,
      "modules": [
        {"type": "platform", "col": 5, "row": 5},
        {"type": "ramp", "col": 5, "row": 4, "rotation": 0},
        {"type": "wall", "col": 0, "row": 0, "rotation": 180}
      ]
    }

col counts along +x and row along +y, both from 0 at the -x, -y corner.
The modules are the parts of the CAD's MODULES part studio:

    platform    1 inch platform, which a ramp climbs onto
    platform_2  2 inch platform, too high for a ramp, so an obstacle
    ramp        1 inch curved ramp
    wall        1 x 4 inch wall, 2 inches high
    thwomp      the thwomp, 1 x 4 inches and 6 high, as a fixed block

rotation is in degrees anticlockwise seen from above. At 0 a ramp rises
toward +y, at 90 toward -x, at 180 toward -y and at 270 toward +x. A wall
or thwomp pegs into its floor tile on the line of holes 1 inch from the
middle, on the side its rotation points to: at 0 along the +y side.

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

from robogame_sim.terrain import Terrain, read_stl, sdf_triangles

MODULE = 0.1016
SIZE = 12
ROTATIONS = (0, 90, 180, 270)
CAD_MODEL = "robogame_arena_cad"
# The peg holes are on a 2 inch grid, 1 inch either side of a tile's middle.
PEG_LINE = 0.0254


class Part:
    """How one CAD part stands on a square. mount is "floor" for the tile,
    "square" for a module centred on its square, and "side" for one on the
    peg line on the side its rotation points to. Heights, and where a side
    part is in the part studio, come from its mesh, so a changed part is
    placed right after a new export without changes here."""

    def __init__(self, label, mesh, mount, colour, turns=False,
                 mesh_collision=False):
        self.label = label
        self.mesh = mesh
        self.mount = mount
        self.colour = colour
        self.turns = turns
        self.mesh_collision = mesh_collision


FLOOR = Part("floor", "floor_module.stl", "floor", (0.647, 0.647, 0.647))
PARTS = {
    "platform": Part('1" platform', "platform_module_1.stl", "square",
                     (0.231, 0.380, 0.706)),
    "platform_2": Part('2" platform', "platform_module_2.stl", "square",
                       (0.165, 0.290, 0.560)),
    "ramp": Part("ramp", "ramp_module_curved.stl", "square",
                 (0.616, 0.812, 0.929), turns=True, mesh_collision=True),
    "wall": Part("wall", "wall_module.stl", "side",
                 (0.918, 0.918, 0.918), turns=True),
    "thwomp": Part("thwomp", "thwomp_module.stl", "side",
                   (0.769, 0.886, 0.953), turns=True),
}
TYPES = tuple(PARTS)

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
    # The installed meshes first, since those are the ones Gazebo loads.
    for root in (share_dir(), source_dir()):
        if root:
            meshes = os.path.join(root, "models", CAD_MODEL, "meshes")
            if all(os.path.isfile(os.path.join(meshes, part.mesh))
                   for part in (FLOOR, *PARTS.values())):
                return meshes

    raise FileNotFoundError(
        f"no {CAD_MODEL} meshes for every module, run "
        f"scripts/export_onshape_arena.py")


BOUNDS = {}


def mesh_bounds(mesh):
    """Lowest and highest corner of a mesh, in the part studio's frame."""
    if mesh not in BOUNDS:
        points = read_stl(os.path.join(mesh_dir(), mesh)).reshape(-1, 3)
        BOUNDS[mesh] = (points.min(axis=0), points.max(axis=0))

    return BOUNDS[mesh]


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
        if PARTS[kind].turns:
            rotation = int(module.get("rotation", 0)) % 360
            if rotation not in ROTATIONS:
                raise ValueError(f"{kind} rotation must be one of "
                                 f"{ROTATIONS}")
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


def modules_info():
    """The modules for the editor: label, footprint and height, in metres."""
    info = {}

    for kind, part in PARTS.items():
        lo, hi = mesh_bounds(part.mesh)
        info[kind] = {"label": part.label, "turns": part.turns,
                      "mount": part.mount,
                      "width": float(hi[1] - lo[1] if part.mount == "side"
                                     else hi[0] - lo[0]),
                      "depth": float(hi[0] - lo[0] if part.mount == "side"
                                     else hi[1] - lo[1]),
                      "height": float(hi[2] - lo[2])}

    return {"module": MODULE, "size": SIZE, "peg_line": PEG_LINE,
            "types": info}


# -- geometry --------------------------------------------------------------

def centre(layout, col, row):
    half = (layout["size"] - 1) / 2
    return (col - half) * MODULE, (row - half) * MODULE


def uphill(rotation):
    """Unit vector a ramp at this rotation rises along, and the side a wall
    or thwomp at it stands on."""
    angle = math.radians(rotation)
    return -math.sin(angle), math.cos(angle)


def placement(part, x, y, rotation):
    """Pose (x, y, z, yaw) of a part's mesh for a square centred on x, y,
    with the floor's top at z = 0, and its bounds' middle in the world."""
    lo, hi = mesh_bounds(part.mesh)
    middle = (lo + hi) / 2

    if part.mount == "floor":
        yaw, z = 0.0, -hi[2]
    else:
        yaw, z = math.radians(rotation), -lo[2]

    if part.mount == "side":
        # The part studio has the part beside the tile's middle; turn it so
        # it is on the side the rotation points to, and move it onto the
        # peg line there.
        ux, uy = uphill(rotation)
        yaw = math.atan2(uy, ux) - math.atan2(middle[1], middle[0])
        c, s = math.cos(yaw), math.sin(yaw)
        mx, my = c * middle[0] - s * middle[1], s * middle[0] + c * middle[1]
        x, y = x + PEG_LINE * ux - mx, y + PEG_LINE * uy - my

    c, s = math.cos(yaw), math.sin(yaw)
    world_middle = (x + c * middle[0] - s * middle[1],
                    y + s * middle[0] + c * middle[1], z + middle[2])

    return (x, y, z, yaw), world_middle


def model_sdf(layout, name="arena"):
    """The arena as an SDF <model>, meshes as model://robogame_arena_cad."""
    parts = []

    def add(label, part, x, y, rotation):
        (px, py, pz, yaw), (bx, by, bz) = placement(part, x, y, rotation)
        uri = f"model://{CAD_MODEL}/meshes/{part.mesh}"
        pose = f"{px:.5f} {py:.5f} {pz:.5f} 0 0 {yaw:.5f}"
        r, g, b = part.colour
        material = (f"<material><ambient>{r} {g} {b} 1</ambient>"
                    f"<diffuse>{r} {g} {b} 1</diffuse>"
                    f"<specular>0.1 0.1 0.1 1</specular></material>")

        if part.mesh_collision:
            collision = (f"<pose>{pose}</pose><geometry><mesh><uri>{uri}"
                         f"</uri></mesh></geometry>")
        else:
            lo, hi = mesh_bounds(part.mesh)
            sx, sy, sz = (float(v) for v in hi - lo)
            collision = (f"<pose>{bx:.5f} {by:.5f} {bz:.5f} 0 0 {yaw:.5f}"
                         f"</pose><geometry><box><size>{sx:.5f} {sy:.5f} "
                         f"{sz:.5f}</size></box></geometry>")

        parts.append(
            f'      <visual name="{label}_visual"><pose>{pose}</pose>'
            f"<geometry><mesh><uri>{uri}</uri></mesh></geometry>{material}"
            f"</visual>\n"
            f'      <collision name="{label}">{collision}</collision>')

    size = layout["size"]
    for row in range(size):
        for col in range(size):
            add(f"floor_{col}_{row}", FLOOR, *centre(layout, col, row), 0)

    for m in layout["modules"]:
        add(f"{m['type']}_{m['col']}_{m['row']}", PARTS[m["type"]],
            *centre(layout, m["col"], m["row"]), m.get("rotation", 0))

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

def blocks(module, dx, dy):
    """Whether a module on the square below a ramp's low end, the ramp
    rising along dx, dy, is in the way of driving onto it."""
    if PARTS[module["type"]].mount != "side":
        return True

    # Only a wall or thwomp on the side next to the ramp.
    sx, sy = (round(v) for v in uphill(module["rotation"]))
    return (sx, sy) == (dx, dy)


def check(layout, ground=None):
    """Problems with a layout that a robot would run into, as dicts with
    type, col, row and message."""
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
        elif low in at and blocks(at[low], dx, dy):
            problems.append(dict(m, message=f"its low end runs into a "
                                 f"{PARTS[at[low]['type']].label}"))

        above = at.get(high, {}).get("type")
        if above == "platform_2":
            problems.append(dict(m, message='its high end meets a 2" '
                                 "platform, an inch above the top of the "
                                 "ramp"))
        elif above != "platform":
            problems.append(dict(m, message="its high end leads to no "
                                 "platform"))

    ground = ground or terrain(layout)
    floor = main_floor(ground)

    if floor is None:
        return problems

    # 2" platforms are obstacles, since no ramp is high enough for them.
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


def open_spot(ground, x, y, taken=(), room=0.08, apart=0.2):
    """The spot nearest x, y on level ground of the main floor (which takes
    in the platforms ramps lead to) with room to turn, at least apart from
    the spots in taken, so a robot is not spawned on a ramp, or wedged
    against a module it then cannot plan its way out of.
    x, y itself when it will do, or when there is no floor at all."""
    floor = main_floor(ground)

    if floor is None:
        return x, y

    # Off the ramps and the line-up zone round them too, where a robot
    # would roll and the planner only lets it drive along the slope.
    # And at the height the spot is at, so a robot meant for the floor
    # is not put up on a platform, when there is anywhere else.
    under = ground.height_at(x, y)
    under = 0.0 if math.isnan(under) else under
    same = np.abs(np.nan_to_num(ground.height, nan=np.inf) - under) < 0.005
    cx, cy = ground.centres()
    apart_from = np.ones(ground.shape, dtype=bool)
    for tx, ty in taken:
        apart_from &= np.hypot(cx - tx, cy - ty) >= apart

    level = floor & ~np.any(ground.axis != 0, axis=-1)
    open_cells = None

    for choice in (level & same & (ground.clearance >= room), level & same,
                   level & (ground.clearance >= room), level, floor):
        if (choice & apart_from).any():
            open_cells = choice & apart_from
            break

    if open_cells is None:
        return x, y

    cell = ground.cell(x, y)
    if ground.inside(cell) and open_cells[cell]:
        return x, y

    distance = np.where(open_cells, np.hypot(cx - x, cy - y), np.inf)
    i, j = np.unravel_index(np.argmin(distance), distance.shape)
    return float(cx[i, j]), float(cy[i, j])


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
