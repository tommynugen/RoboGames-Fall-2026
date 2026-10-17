"""Arena layouts: which modules stand where on the arena floor.

The arena is a square grid of 4 inch (0.1016 m) modules, 12 x 12, centred
on the origin. Every square has a floor tile. A layout lists what stands on
them:

    {
      "size": 12,
      "modules": [
        {"type": "platform", "col": 5, "row": 5},
        {"type": "ramp", "col": 5, "row": 4, "rotation": 0},
        {"type": "wall", "col": 0.5, "row": 0, "rotation": 180}
      ]
    }

col counts along +x and row along +y, both from 0 at the -x, -y corner.
The modules are the parts of the CAD's MODULES part studio:

    platform    1 inch platform, which a ramp climbs onto
    platform_2  2 inch platform, too high for a ramp, so an obstacle
    ramp        1 inch curved ramp
    wall        1 x 4 inch wall, 2 inches high
    thwomp      the thwomp, 1 x 4 inches and 6 high, as a fixed block

Every module pegs into the same 2 inch grid of holes, which runs on
across the joins between floor tiles, so a module goes anywhere on that
grid: col and row are in steps of a half, and a module at col 5.5 stands
across the tiles at cols 5 and 6. The holes split the floor into 2 inch
cells, one hole in the middle of each and four to a tile. A platform or
ramp covers 2 x 2 cells and a wall or thwomp 1 x 2, and no two modules
share a cell, so two walls fit side by side on one tile.

A platform can carry more modules: a module with "level": 1 stands on
platforms at level 0 under every cell it covers, one at level 2 on
platforms at level 1, and so on, all of them the same height. Ramps,
walls, thwomps and platforms all stack on either platform, so a ramp on a
1 inch platform climbs to a 2 inch one. level is left out for modules on
the floor.

rotation is in degrees anticlockwise seen from above. At 0 a ramp rises
toward +y, at 90 toward -x, at 180 toward -y and at 270 toward +x. A wall
or thwomp pegs in on the line of holes 1 inch from the middle of its
square, on the side its rotation points to: at 0 along the +y side.

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
# What other modules can stand on.
PLATFORMS = ("platform", "platform_2")

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

    for module in layout.get("modules", []):
        kind = module.get("type")
        if kind not in TYPES:
            raise ValueError(f"unknown module type {kind!r}")

        entry = {"type": kind, "col": half(module.get("col"), "col"),
                 "row": half(module.get("row"), "row")}
        level = int(module.get("level", 0))
        if level < 0:
            raise ValueError(f"{where(entry)} is below the floor")
        if level:
            entry["level"] = level
        if PARTS[kind].turns:
            rotation = int(module.get("rotation", 0)) % 360
            if rotation not in ROTATIONS:
                raise ValueError(f"{kind} rotation must be one of "
                                 f"{ROTATIONS}")
            entry["rotation"] = rotation
        modules.append(entry)

    taken = {}
    for m in modules:
        level = m.get("level", 0)
        for cell in cells(m):
            if not on_grid(cell, size):
                raise ValueError(f"{where(m)} is off the grid")
            other = taken.setdefault((cell, level), m)
            if other is not m:
                raise ValueError(f"{where(m)} overlaps the {where(other)}")

    # Only platforms carry anything, and all at one height, so it stands
    # level.
    for m in modules:
        level = m.get("level", 0)
        if not level:
            continue
        if any(taken.get((cell, level - 1), {}).get("type") not in PLATFORMS
               for cell in cells(m)):
            raise ValueError(f"{where(m)} is not on a platform")

    at = columns(modules)
    for m in modules:
        if len({round(stand(at, m, cell), 4) for cell in cells(m)}) > 1:
            raise ValueError(f"{where(m)} stands across platforms of "
                             f"different heights")

    modules.sort(key=lambda m: (m["row"], m["col"], m.get("level", 0),
                                m["type"], m.get("rotation", 0)))
    return {"size": size, "modules": modules}


def half(value, name):
    """A col or row, which is in steps of a half: a whole number when it is
    one, so layouts keep reading 5 and not 5.0."""
    doubled = float(value) * 2
    if doubled != round(doubled):
        raise ValueError(f"{name} must be in steps of a half, not {value}")

    doubled = round(doubled)
    return doubled // 2 if doubled % 2 == 0 else doubled / 2


def where(m):
    """A module and where it is, for messages."""
    text = f"{m['type']} at col {m['col']}, row {m['row']}"
    if m.get("level"):
        text += f", level {m['level']}"
    return text


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


def part_height(kind):
    lo, hi = mesh_bounds(PARTS[kind].mesh)
    return float(hi[2] - lo[2])


def cells(m):
    """The 2 inch cells a module covers, as (x, y) counted like col and row
    but two to a square: square (col, row) is cells 2 col and 2 col + 1
    along x, and the same along y."""
    x, y = round(2 * m["col"]), round(2 * m["row"])

    if PARTS[m["type"]].mount != "side":
        return [(x, y), (x + 1, y), (x, y + 1), (x + 1, y + 1)]

    # The half of the square on the side the module faces.
    ux, uy = (round(v) for v in uphill(m["rotation"]))
    if ux == 0:
        y += uy > 0
        return [(x, y), (x + 1, y)]

    x += ux > 0
    return [(x, y), (x, y + 1)]


def on_grid(cell, size):
    return all(0 <= v < 2 * size for v in cell)


def shifted(found, dx, dy):
    return [(x + dx, y + dy) for x, y in found]


def columns(modules):
    """The modules over each cell, bottom first, keyed by cell."""
    found = {}

    for m in modules:
        for cell in cells(m):
            found.setdefault(cell, []).append(m)

    for column in found.values():
        column.sort(key=lambda m: m.get("level", 0))

    return found


def stand(at, m, cell=None):
    """How high above the floor a module stands: the platforms under it,
    on the given cell or its first."""
    level = m.get("level", 0)
    return sum(part_height(p["type"])
               for p in at[cell or cells(m)[0]] if p.get("level", 0) < level)


def surface(column):
    """Height of the top platform of a cell's column, which a module on top
    or a robot stands on, and the module standing there, if any."""
    height = 0.0

    for m in column or ():
        if m["type"] not in PLATFORMS:
            return height, m
        height += part_height(m["type"])

    return height, None


def placement(part, x, y, rotation, base=0.0):
    """Pose (x, y, z, yaw) of a part's mesh for a square centred on x, y,
    with the floor's top at z = 0 and the part standing base above it, and
    its bounds' middle in the world."""
    lo, hi = mesh_bounds(part.mesh)
    middle = (lo + hi) / 2

    if part.mount == "floor":
        yaw, z = 0.0, -hi[2]
    else:
        yaw, z = math.radians(rotation), base - lo[2]

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

    def add(label, part, x, y, rotation, base=0.0):
        (px, py, pz, yaw), (bx, by, bz) = placement(part, x, y, rotation,
                                                    base)
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

    at = columns(layout["modules"])
    names = set()
    for m in layout["modules"]:
        level = m.get("level", 0)
        name = (f"{m['type']}_{m['col']}_{m['row']}".replace(".", "p")
                + (f"_{level}" if level else ""))
        # Walls side by side on one square.
        if name in names:
            name += f"_r{m['rotation']}"
        names.add(name)
        add(name, PARTS[m["type"]], *centre(layout, m["col"], m["row"]),
            m.get("rotation", 0), stand(at, m))

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

def facing(module):
    """The side, as (dx, dy), a ramp rises toward or a wall or thwomp
    stands on."""
    dx, dy = uphill(module["rotation"])
    return round(dx), round(dy)


def label(module):
    return PARTS[module["type"]].label


def in_the_way(module):
    return ("the side of a ramp" if module["type"] == "ramp"
            else f"a {label(module)}")


def along(ramp, cell):
    """Which row of a ramp's cells a cell is, 0 at the bottom, 1 at the
    top."""
    dx, dy = facing(ramp)
    rows = [x * dx + y * dy for x, y in cells(ramp)]
    return cell[0] * dx + cell[1] * dy - min(rows)


def edge(at, cell, side):
    """What a robot meets crossing the side (dx, dy) of a cell, as
    (height, blocker). height is that of the cell's top along the side,
    None where it cannot be crossed: along the slope of a ramp, or across a
    wall or thwomp, which is then the blocker."""
    base, top = surface(at.get(cell))

    if top is None:
        return base, None

    if top["type"] == "ramp":
        dx, dy = facing(top)
        row = along(top, cell)
        if side == (dx, dy):
            return base + part_height("ramp") * (row + 1) / 2, None
        if side == (-dx, -dy):
            return base + part_height("ramp") * row / 2, None
        return None, top

    # A wall or thwomp stands across the middle of its cells, so it closes
    # them on either side of it, and leaves the way along it open.
    if facing(top) in (side, (-side[0], -side[1])):
        return None, top

    return base, None


def crossable(at, a, b):
    """Whether a robot can drive between side by side cells a and b."""
    top_a, top_b = surface(at.get(a))[1], surface(at.get(b))[1]
    # Within one module.
    if top_a is not None and top_a is top_b:
        return True

    side = (b[0] - a[0], b[1] - a[1])
    height_a, blocker_a = edge(at, a, side)
    height_b, blocker_b = edge(at, b, (-side[0], -side[1]))

    if height_a is not None and height_b is not None:
        return same_height(height_a, height_b)

    # Ramps side by side, rising the same way from the same height and
    # level with each other, are one wider ramp.
    return (blocker_a is not None and blocker_b is not None
            and blocker_a["type"] == blocker_b["type"] == "ramp"
            and blocker_a["rotation"] == blocker_b["rotation"]
            and along(blocker_a, a) == along(blocker_b, b)
            and same_height(surface(at[a])[0], surface(at[b])[0]))


def inches(height):
    n = round(height / 0.0254)
    return "an inch" if n == 1 else f"{n} inches"


def same_height(a, b):
    return abs(a - b) < 1e-3


def check(layout):
    """Problems with a layout that a robot would run into, as the modules
    they are with, and a message.

    They are worked out module by module, from the height of each cell's
    top along its sides, not from chase_node's terrain: that keeps the
    robot's centre clear enough of every edge to turn on the spot, so it
    finds the one module landings a chain of ramps turns on too narrow,
    though a robot drives over them."""
    layout = normalise(layout)
    size = layout["size"]
    at = columns(layout["modules"])
    rise = part_height("ramp")
    problems = []
    # Level cells ramps lead onto.
    ramp_tops = set()

    for m in layout["modules"]:
        if m["type"] != "ramp":
            continue

        base = stand(at, m)
        dx, dy = facing(m)
        own = cells(m)
        # The row of cells in front of each end.
        low = [cell for cell in shifted(own, -dx, -dy) if cell not in own]
        high = [cell for cell in shifted(own, dx, dy) if cell not in own]

        if not all(on_grid(cell, size) for cell in low):
            problems.append(dict(m, message="its low end is at the edge of "
                                 "the arena, so it cannot be driven onto"))
        else:
            # The side of the cells in front facing the ramp, which may be
            # the top of another ramp leading up to this one.
            ahead = [(cell, *edge(at, cell, (dx, dy))) for cell in low]
            blockers = [b for _, _, b in ahead if b is not None]
            higher = [cell for cell, height, b in ahead
                      if b is None and height > base + 1e-3]
            lower = [height for _, height, b in ahead
                     if b is None and height < base - 1e-3]

            if blockers:
                problems.append(dict(m, message=f"its low end runs into "
                                     f"{in_the_way(blockers[0])}"))
            elif higher:
                low_top = surface(at[higher[0]])[1]
                under = (low_top if low_top is not None else
                         [p for p in at[higher[0]]
                          if p["type"] in PLATFORMS][-1])
                problems.append(dict(m, message=f"its low end runs into a "
                                     f"{label(under)}"))
            elif lower:
                problems.append(dict(m, message=f"its low end is "
                                     f"{inches(base - min(lower))} above "
                                     f"the ground in front of it"))

        top = base + rise
        reached, above, blockers = [], [], []
        for cell in high:
            if not on_grid(cell, size):
                continue

            height, blocker = edge(at, cell, (-dx, -dy))
            high_top = surface(at[cell])[1] if cell in at else None
            platforms = [p for p in at.get(cell, ())
                         if p["type"] in PLATFORMS]

            if blocker is not None:
                blockers.append(blocker)
            elif same_height(height, top):
                # Onto a platform, or up another ramp from the top of this
                # one.
                reached.append(cell)
                if high_top is None or high_top["type"] != "ramp":
                    ramp_tops.add(cell)
            elif height > top and platforms:
                above.append((height, high_top if high_top is not None
                              and high_top["type"] == "ramp"
                              else platforms[-1]))

        if len(reached) == len(high):
            continue
        if blockers:
            problems.append(dict(m, message=f"its high end runs into "
                                 f"{in_the_way(blockers[0])}"))
        elif above:
            height, what = max(above, key=lambda a: a[0])
            problems.append(dict(m, message=f"its high end meets a "
                                 f"{label(what)}, {inches(height - top)} "
                                 f"above the top of the ramp"))
        elif reached:
            problems.append(dict(m, message="part of its high end leads to "
                                 "no platform"))
        else:
            problems.append(dict(m, message="its high end leads to no "
                                 "platform"))

    floor = floor_cells(at, size)

    # The tops robots are meant to get onto: 1 inch platforms, and any
    # other height a ramp leads to. The rest, like a 2 inch platform no
    # ramp reaches, are obstacles.
    for region in platform_regions(at):
        height = region[0][1]
        if not (same_height(height, part_height("platform"))
                or any(cell in ramp_tops for cell, _, _ in region)):
            continue

        stranded = []
        for cell, _, m in region:
            # Where a wall or thwomp stands on it is not somewhere to get
            # to.
            if (cell in floor or surface(at[cell])[1] is not None
                    or any(m is s for s in stranded)):
                continue
            stranded.append(m)
            problems.append(dict(m, message="no robot can get onto this "
                                 "platform from the floor"))

    return problems


def floor_cells(at, size):
    """The cells a robot on the floor can drive to, which are those joined
    to the largest stretch of bare floor."""
    seen, best, best_bare = set(), set(), 0

    for start in ((x, y) for y in range(2 * size) for x in range(2 * size)):
        if start in seen:
            continue

        region, todo = {start}, [start]
        seen.add(start)

        while todo:
            x, y = todo.pop()

            for near in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if (on_grid(near, size) and near not in seen
                        and crossable(at, (x, y), near)):
                    seen.add(near)
                    region.add(near)
                    todo.append(near)

        bare = sum(1 for cell in region
                   if not any(p["type"] in PLATFORMS or p["type"] == "ramp"
                              for p in at.get(cell, ())))

        if bare > best_bare:
            best, best_bare = region, bare

    return best


def platform_regions(at):
    """Cells whose top is a platform, in side by side groups of the same
    height, as lists of (cell, height, the platform on top)."""
    tops = {}

    for cell, column in at.items():
        height, m = surface(column)
        platforms = [p for p in column if p["type"] in PLATFORMS]
        # A wall or thwomp stands on the edge, so the platform is still
        # there to drive on.
        if platforms and (m is None or PARTS[m["type"]].mount == "side"):
            tops[cell] = height, platforms[-1]

    regions, seen = [], set()

    for cell in sorted(tops):
        if cell in seen:
            continue

        region, todo = [], [cell]
        seen.add(cell)

        while todo:
            x, y = todo.pop()
            region.append(((x, y), *tops[(x, y)]))

            for near in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if (near in tops and near not in seen
                        and same_height(tops[near][0], tops[cell][0])):
                    seen.add(near)
                    todo.append(near)

        regions.append(region)

    return regions


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
