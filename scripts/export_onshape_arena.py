#!/usr/bin/env python3
"""Exports the Onshape arena assembly into a static Gazebo model.

    scripts/export_onshape_arena.py

Reads the API keys from ~/.config/onshape/keys.env (ONSHAPE_ACCESS_KEY and
ONSHAPE_SECRET_KEY, or the same names in the environment), downloads every
distinct part of the MODULES part studio once as an STL in metres, and
writes src/robogame_sim/models/robogame_arena_cad with one visual per
module, placed where MAIN ASSEMBLY puts it. Parts the assembly does not
place are exported too, since the arena editor's layouts are built from
them. The quadrant plates, the camera frame and the hardware are left out.
Run it again after changing the CAD, then rebuild.

The model is moved so the middle of the floor tiles is at the origin and
their top at z = 0, which is where the robots spawn. Modules away from the
tiles, such as ones left where they were inserted, are left out.
"""

import base64
import json
import math
import os
import re
import shutil
import struct
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

API = "https://cad.onshape.com/api/v10"
DOCUMENT = "a45ddd958f6e1eddbc023de1"
WORKSPACE = "bac0a0f81de971f1870aaa2d"
ASSEMBLY = "bb81c7b3a8b24aa971e36ea5"
MODULES = "c795788e510a7a771142e369"

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_NAME = "robogame_arena_cad"
MODEL_DIR = os.path.join(HERE, "src", "robogame_sim", "models", MODEL_NAME)

# Coarse enough to keep the hole patterns on the MAXTube from running into
# megabytes, fine enough that the ramps stay smooth.
STL_OPTIONS = {"angleTolerance": 0.3, "chordTolerance": 0.001}

# Curved parts collide with their mesh, everything else with its box.
MESH_COLLISION = ("RAMP",)


def load_keys():
    path = os.path.expanduser("~/.config/onshape/keys.env")
    keys = {}

    if os.path.exists(path):
        for line in open(path):
            name, _, value = line.strip().partition("=")
            keys[name] = value

    access = os.environ.get("ONSHAPE_ACCESS_KEY", keys.get("ONSHAPE_ACCESS_KEY"))
    secret = os.environ.get("ONSHAPE_SECRET_KEY", keys.get("ONSHAPE_SECRET_KEY"))

    if not access or not secret:
        sys.exit(f"no Onshape API keys, put them in {path}")

    return base64.b64encode(f"{access}:{secret}".encode()).decode()


AUTH = None


def get(path, params=None, accept="application/json"):
    url = API + path + ("?" + urllib.parse.urlencode(params) if params else "")
    request = urllib.request.Request(
        url, headers={"Authorization": "Basic " + AUTH, "Accept": accept})

    for attempt in range(8):
        try:
            return urllib.request.urlopen(request).read()
        except urllib.error.HTTPError as error:
            # The API allows only a few exports a second.
            if error.code != 429:
                raise
            time.sleep(int(error.headers.get("Retry-After") or 10 * (attempt + 1)))

    raise RuntimeError(f"still rate limited on {path}")


def read_stl(data):
    count = struct.unpack_from("<I", data, 80)[0]
    return [
        struct.unpack_from("<9f", data, 84 + 50 * k + 12)
        for k in range(count)
    ]


def bounds(points):
    return (
        [min(p[d] for p in points) for d in range(3)],
        [max(p[d] for p in points) for d in range(3)],
    )


def apply(transform, point):
    x, y, z = point
    return [
        transform[4 * r] * x + transform[4 * r + 1] * y
        + transform[4 * r + 2] * z + transform[4 * r + 3]
        for r in range(3)
    ]


def rpy(transform):
    """Roll, pitch, yaw of the rotation part, in SDF's convention."""
    r = [[transform[4 * i + j] for j in range(3)] for i in range(3)]
    pitch = math.asin(max(-1.0, min(1.0, -r[2][0])))

    if abs(r[2][0]) < 0.9999:
        roll = math.atan2(r[2][1], r[2][2])
        yaw = math.atan2(r[1][0], r[0][0])
    else:
        roll = math.atan2(-r[1][2], r[1][1])
        yaw = 0.0

    return roll, pitch, yaw


def pose(transform, shift):
    x, y, z = (transform[3] + shift[0], transform[7] + shift[1],
               transform[11] + shift[2])
    roll, pitch, yaw = rpy(transform)
    return f"{x:.5f} {y:.5f} {z:.5f} {roll:.5f} {pitch:.5f} {yaw:.5f}"


def colour(appearance):
    if appearance:
        return tuple(appearance[c] / 255.0 for c in ("red", "green", "blue"))

    return (0.7, 0.7, 0.7)


def material(rgb):
    r, g, b = rgb
    return (
        f"<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>"
        f"<diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>"
        f"<specular>0.1 0.1 0.1 1</specular></material>"
    )


def main():
    global AUTH
    AUTH = load_keys()

    assembly = json.loads(get(
        f"/assemblies/d/{DOCUMENT}/w/{WORKSPACE}/e/{ASSEMBLY}",
        {"includeMateFeatures": "false"}))
    root = assembly["rootAssembly"]
    instances = {i["id"]: i for i in root["instances"]}

    mesh_dir = os.path.join(MODEL_DIR, "meshes")
    shutil.rmtree(MODEL_DIR, ignore_errors=True)
    os.makedirs(mesh_dir)

    # One STL per part and configuration, shared by all its instances.
    # Every part of the studio is exported, placed or not, since
    # arena_layout builds layouts out of any of them.
    parts = {}
    appearances = {}
    studio = f"/d/{DOCUMENT}/w/{WORKSPACE}/e/{MODULES}"
    library = [
        {"elementId": MODULES, "partId": p["partId"], "name": p["name"]}
        for p in json.loads(get(f"/parts{studio}"))
    ]
    placed_instances = [
        i for i in root["instances"]
        if i["type"] == "Part" and not i["suppressed"]
        and i["elementId"] == MODULES and i["documentId"] == DOCUMENT
    ]

    for instance in library + placed_instances:
        config = instance.get("fullConfiguration", "default")
        key = (instance["elementId"], instance["partId"], config)

        if key in parts:
            continue

        name = instance["name"].rsplit(" <", 1)[0]
        slug = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")

        if any(p["slug"] == slug for p in parts.values()):
            slug += "_" + re.sub(r"[^a-z0-9]", "", instance["partId"].lower())

        params = dict(STL_OPTIONS, partIds=instance["partId"], units="meter",
                      mode="binary", grouping="true")

        if config != "default":
            params["configuration"] = config

        data = get(f"/partstudios{studio}/stl", params,
                   accept="application/octet-stream")
        open(os.path.join(mesh_dir, slug + ".stl"), "wb").write(data)

        if studio not in appearances:
            listing = json.loads(get(
                f"/parts{studio}",
                {"configuration": config} if config != "default" else None))
            appearances[studio] = {
                p["partId"]: p.get("appearance", {}).get("color")
                for p in listing
            }

        triangles = read_stl(data)
        parts[key] = {
            "name": name,
            "slug": slug,
            "local": bounds([t[i:i + 3] for t in triangles for i in (0, 3, 6)]),
            "points": [t[i:i + 3] for t in triangles[::5] for i in (0, 3, 6)],
            "colour": colour(appearances[studio].get(instance["partId"])),
        }
        print(f"{slug:50s} {len(triangles):7d} triangles "
              f"{len(data) / 1024:6.0f} KiB")

    placed = []

    for occurrence in root["occurrences"]:
        instance = instances[occurrence["path"][-1]]
        key = (instance["elementId"], instance["partId"],
               instance.get("fullConfiguration", "default"))

        if key not in parts or occurrence.get("hidden"):
            continue

        part = parts[key]
        world = bounds([apply(occurrence["transform"], p) for p in part["points"]])
        placed.append((instance, part, occurrence["transform"], world))

    # The floor tiles set the arena: the most common tile height is the
    # floor, and the tiles at that height its footprint. A tile left where
    # it was inserted sits at some other height.
    heights = [round(w[1][2], 4) for i, _, _, w in placed
               if i["name"].startswith("FLOOR MODULE")]
    floor = max(set(heights), key=heights.count)
    tiles = [w for i, _, _, w in placed
             if i["name"].startswith("FLOOR MODULE")
             and round(w[1][2], 4) == floor]
    tile_lo, tile_hi = bounds([b for w in tiles for b in w])
    centre = [(tile_lo[d] + tile_hi[d]) / 2 for d in range(2)]
    shift = (-centre[0], -centre[1], -floor)

    # Walls may stand on the edge of the tiles.
    def inside(world, margin=0.05):
        mid = [(world[0][d] + world[1][d]) / 2 for d in range(3)]
        return (tile_lo[0] - margin <= mid[0] <= tile_hi[0] + margin
                and tile_lo[1] - margin <= mid[1] <= tile_hi[1] + margin
                and mid[2] >= tile_lo[2])

    print(f"floor tiles {tile_hi[0] - tile_lo[0]:.3f} x "
          f"{tile_hi[1] - tile_lo[1]:.3f} m, {floor - tile_lo[2]:.4f} m "
          f"thick, centre {centre[0]:+.3f} {centre[1]:+.3f}, top at z {floor:+.3f}")

    elements = []
    skipped = []

    for index, (instance, part, transform, world) in enumerate(placed):
        if not inside(world):
            skipped.append(instance["name"])
            continue

        where = pose(transform, shift)
        name = re.sub(r"[^A-Za-z0-9]+", "_", instance["name"]).strip("_")
        mesh = (f"<geometry><mesh><uri>meshes/{part['slug']}.stl</uri>"
                f"</mesh></geometry>")

        elements.append(
            f'      <visual name="{name}_{index}">\n'
            f"        <pose>{where}</pose>\n"
            f"        {mesh}\n"
            f"        {material(part['colour'])}\n"
            f"      </visual>")

        if any(k in instance["name"] for k in MESH_COLLISION):
            geometry = mesh
        else:
            # A box in the part's own frame, placed like the part.
            lo, hi = part["local"]
            middle = [(lo[d] + hi[d]) / 2 for d in range(3)]
            offset = list(transform)

            for r in range(3):
                offset[4 * r + 3] = apply(transform, middle)[r]

            where = pose(offset, shift)
            size = " ".join(f"{hi[d] - lo[d]:.5f}" for d in range(3))
            geometry = f"<geometry><box><size>{size}</size></box></geometry>"

        elements.append(
            f'      <collision name="{name}_{index}">\n'
            f"        <pose>{where}</pose>\n"
            f"        {geometry}\n"
            f"      </collision>")

    open(os.path.join(MODEL_DIR, "model.sdf"), "w").write(
        '<?xml version="1.0" ?>\n'
        "<!-- Written by scripts/export_onshape_arena.py, do not edit. -->\n"
        '<sdf version="1.9">\n'
        f'  <model name="{MODEL_NAME}">\n'
        "    <static>true</static>\n"
        '    <link name="link">\n'
        + "\n".join(elements) + "\n"
        "    </link>\n"
        "  </model>\n"
        "</sdf>\n")

    open(os.path.join(MODEL_DIR, "model.config"), "w").write(
        '<?xml version="1.0" ?>\n'
        "<model>\n"
        f"  <name>{MODEL_NAME}</name>\n"
        "  <version>1.0</version>\n"
        '  <sdf version="1.9">model.sdf</sdf>\n'
        "  <description>RoboGame Arena from Onshape, "
        f"document {DOCUMENT}</description>\n"
        "</model>\n")

    print(f"{len(placed) - len(skipped)} instances written to {MODEL_DIR}")

    for name in skipped:
        print(f"  left out, outside the arena: {name}")


if __name__ == "__main__":
    main()
