"""The arena's terrain, and paths over it for chase_node. No ROS in here.

The collision geometry of the arena model is rasterised into a height map:
the top of every box, and the upward faces of every mesh. Wherever the
height jumps between neighbouring cells, as at a platform's edge, a ramp's
sides or the edge of the floor tiles, is a step the wheels cannot climb,
and the robot's centre is kept CLEARANCE away from it. Where the ground
slopes, on a ramp, the robot may only drive up or down the slope, and that
holds for LINE_UP around it too, so it lines up before climbing and comes
off the top straight.

A path from the hunter to the target is planned over the cells with A*,
then pulled straight, so the hunter goes round a platform that is in the
way and up a ramp to reach a target on top of one.
"""

import heapq
import math
import os
import re
import struct
import xml.etree.ElementTree as ElementTree

import numpy as np

from robogame_sim.robot_sdf import CHASSIS, WHEEL_WIDTH, WHEEL_Y

RESOLUTION = 0.02
# Height change between neighbouring cells that is a step, not a slope.
# The arena ramps rise at most about 0.008 m per cell, the platforms 0.025.
STEP = 0.013
# Gradient above which the ground counts as a ramp.
SLOPE = 0.05
# The chassis corners are 0.053 m from its centre, so it can turn on the
# spot at CLEARANCE from a step without touching it. Cells closer than
# SOFT_CLEARANCE cost more, so paths keep a little further off.
CLEARANCE = math.hypot(CHASSIS[0] / 2, WHEEL_Y + WHEEL_WIDTH / 2)
SOFT_CLEARANCE = 0.1
LINE_UP = 0.07
# Moves on and around a ramp must be within 15 degrees of its slope.
ALIGN = math.cos(math.radians(15))

MOVES = [(di, dj) for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj]


def rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def parse_pose(element):
    """(rotation, translation) of an SDF element's <pose>, or identity."""
    pose = element.find("pose") if element is not None else None
    values = [float(v) for v in pose.text.split()] if pose is not None else []
    values += [0.0] * (6 - len(values))

    return rotation(*values[3:]), np.array(values[:3])


def box_top(size):
    """The top face of a box centred on the origin, as two triangles."""
    x, y, z = (s / 2 for s in size)
    corners = [(-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z)]

    return np.array([[corners[0], corners[1], corners[2]],
                     [corners[0], corners[2], corners[3]]])


def read_stl(path):
    """Triangles of a binary or ASCII STL, as an n x 3 x 3 array."""
    with open(path, "rb") as stl:
        data = stl.read()

    if len(data) >= 84:
        count = struct.unpack("<I", data[80:84])[0]

        if len(data) == 84 + 50 * count:
            rows = np.frombuffer(
                data[84:], dtype=np.dtype([("normal", "<f4", 3),
                                           ("vertices", "<f4", (3, 3)),
                                           ("attribute", "<u2")]))
            return rows["vertices"].astype(float)

    numbers = re.findall(
        rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)

    return np.array(numbers, dtype=float).reshape(-1, 3, 3)


def model_triangles(path):
    """Every collision surface in an SDF model, in the model's frame."""
    model_dir = os.path.dirname(os.path.abspath(path))
    root = ElementTree.parse(path).getroot()
    triangles = []

    for link in root.iter("link"):
        link_rotation, link_translation = parse_pose(link)

        for collision in link.iter("collision"):
            local_rotation, local_translation = parse_pose(collision)
            geometry = collision.find("geometry")
            box = geometry.find("box") if geometry is not None else None
            mesh = geometry.find("mesh") if geometry is not None else None

            if box is not None:
                faces = box_top([float(v) for v in
                                 box.find("size").text.split()])
            elif mesh is not None:
                uri = mesh.find("uri").text.strip()
                uri = re.sub(r"^model://[^/]+/", "", uri)
                faces = read_stl(os.path.join(model_dir, uri))
                scale = mesh.find("scale")

                if scale is not None:
                    faces = faces * np.array(
                        [float(v) for v in scale.text.split()])
            else:
                continue

            faces = faces @ local_rotation.T + local_translation
            triangles.append(faces @ link_rotation.T + link_translation)

    return np.concatenate(triangles) if triangles else np.zeros((0, 3, 3))


class Terrain:
    """Height map of the arena, with the cells a robot can drive on."""

    def __init__(self, triangles, resolution=RESOLUTION):
        self.resolution = resolution
        points = triangles.reshape(-1, 3)
        self.origin = points[:, :2].min(axis=0)
        shape = np.ceil(
            (points[:, :2].max(axis=0) - self.origin) / resolution
        ).astype(int)
        self.shape = (int(shape[0]), int(shape[1]))

        self.height = self.rasterise(triangles)
        self.find_steps()
        self.find_slopes()

    @classmethod
    def from_sdf(cls, path, resolution=RESOLUTION):
        return cls(model_triangles(path), resolution)

    # -- building the map -------------------------------------------------

    def centres(self):
        r = self.resolution
        xs = self.origin[0] + (np.arange(self.shape[0]) + 0.5) * r
        ys = self.origin[1] + (np.arange(self.shape[1]) + 0.5) * r
        return np.meshgrid(xs, ys, indexing="ij")

    def rasterise(self, triangles):
        """Highest upward facing surface over each cell centre, NaN where
        there is none."""
        height = np.full(self.shape, np.nan)
        cx, cy = self.centres()
        r = self.resolution

        for a, b, c in triangles:
            normal = np.cross(b - a, c - a)
            normal_z = normal[2]
            area = np.linalg.norm(normal)

            # Walls are not ground. Undersides are kept: they are under
            # the top, so the highest surface still wins.
            if area == 0 or abs(normal_z) < 0.3 * area:
                continue

            lo = np.floor((np.minimum(np.minimum(a, b), c)[:2]
                           - self.origin) / r - 0.5).astype(int)
            hi = np.ceil((np.maximum(np.maximum(a, b), c)[:2]
                          - self.origin) / r - 0.5).astype(int)
            lo = np.clip(lo, 0, None)
            hi = np.minimum(hi, np.array(self.shape) - 1)

            if (hi < lo).any():
                continue

            x = cx[lo[0]:hi[0] + 1, lo[1]:hi[1] + 1]
            y = cy[lo[0]:hi[0] + 1, lo[1]:hi[1] + 1]

            # Barycentric coordinates of each cell centre.
            u = ((b[1] - c[1]) * (x - c[0])
                 + (c[0] - b[0]) * (y - c[1])) / normal_z
            v = ((c[1] - a[1]) * (x - c[0])
                 + (a[0] - c[0]) * (y - c[1])) / normal_z
            w = 1 - u - v
            inside = (u >= -1e-9) & (v >= -1e-9) & (w >= -1e-9)
            z = u * a[2] + v * b[2] + w * c[2]

            patch = height[lo[0]:hi[0] + 1, lo[1]:hi[1] + 1]
            patch[inside] = np.fmax(patch[inside], z[inside])

        return height

    def find_steps(self):
        h = self.height
        step = np.isnan(h)
        padded = np.pad(h, 1, constant_values=np.nan)

        for di, dj in MOVES:
            neighbour = padded[1 + di:1 + di + h.shape[0],
                               1 + dj:1 + dj + h.shape[1]]
            with np.errstate(invalid="ignore"):
                step |= np.isnan(neighbour) | (np.abs(neighbour - h) > STEP)

        cx, cy = self.centres()
        steps = np.stack([cx[step], cy[step]], axis=1)
        self.clearance = np.full(self.shape, np.inf)

        # Distance from each cell centre to the nearest step, in chunks so
        # the arrays stay small.
        flat = np.stack([cx.ravel(), cy.ravel()], axis=1)
        nearest = np.full(len(flat), np.inf)

        for start in range(0, len(steps), 256):
            chunk = steps[start:start + 256]
            d = np.hypot(flat[:, None, 0] - chunk[None, :, 0],
                         flat[:, None, 1] - chunk[None, :, 1])
            nearest = np.minimum(nearest, d.min(axis=1))

        self.clearance = nearest.reshape(self.shape)
        self.clearance[step] = 0.0
        self.free = self.clearance >= CLEARANCE
        self.cost = 1.0 + 4.0 * np.clip(
            (SOFT_CLEARANCE - self.clearance)
            / (SOFT_CLEARANCE - CLEARANCE), 0.0, 1.0)

    def find_slopes(self):
        """The slope direction that moves on and around a ramp must follow,
        as unit vectors, zero where moves are free."""
        h = np.where(np.isnan(self.height), 0.0, self.height)
        gx, gy = np.gradient(h, self.resolution)
        magnitude = np.hypot(gx, gy)
        sloped = self.free & (magnitude > SLOPE)

        cx, cy = self.centres()
        self.axis = np.zeros(self.shape + (2,))

        if not sloped.any():
            return

        sx, sy = cx[sloped], cy[sloped]
        ux, uy = gx[sloped] / magnitude[sloped], gy[sloped] / magnitude[sloped]
        flat = np.stack([cx.ravel(), cy.ravel()], axis=1)
        sum_x = np.zeros(len(flat))
        sum_y = np.zeros(len(flat))

        for start in range(0, len(sx), 256):
            part = slice(start, start + 256)
            near = np.hypot(flat[:, None, 0] - sx[None, part],
                            flat[:, None, 1] - sy[None, part]) <= LINE_UP
            sum_x += near @ ux[part]
            sum_y += near @ uy[part]

        norm = np.hypot(sum_x, sum_y)
        has = norm > 1e-9
        axis = np.zeros((len(flat), 2))
        axis[has, 0] = sum_x[has] / norm[has]
        axis[has, 1] = sum_y[has] / norm[has]
        self.axis = axis.reshape(self.shape + (2,))

    # -- queries ----------------------------------------------------------

    def cell(self, x, y):
        i = int(math.floor((x - self.origin[0]) / self.resolution))
        j = int(math.floor((y - self.origin[1]) / self.resolution))
        return i, j

    def centre(self, cell):
        i, j = cell
        return (self.origin[0] + (i + 0.5) * self.resolution,
                self.origin[1] + (j + 0.5) * self.resolution)

    def inside(self, cell):
        return 0 <= cell[0] < self.shape[0] and 0 <= cell[1] < self.shape[1]

    def height_at(self, x, y):
        cell = self.cell(x, y)
        return self.height[cell] if self.inside(cell) else math.nan

    def aligned(self, cell, dx, dy):
        """Whether moving along dx, dy is allowed in cell."""
        ax, ay = self.axis[cell]

        if ax == 0.0 and ay == 0.0:
            return True

        length = math.hypot(dx, dy)
        return length < 1e-12 or abs(ax * dx + ay * dy) >= ALIGN * length

    def clear_line(self, ax, ay, bx, by):
        """Whether a robot can drive straight from a to b."""
        dx, dy = bx - ax, by - ay
        samples = max(1, int(math.hypot(dx, dy) / (self.resolution / 2)))

        for k in range(samples + 1):
            cell = self.cell(ax + dx * k / samples, ay + dy * k / samples)

            if (not self.inside(cell) or not self.free[cell]
                    or not self.aligned(cell, dx, dy)):
                return False

        return True

    def nearest_free(self, x, y, level, reach=0.15):
        """The free cell nearest x, y on ground within STEP of level."""
        centre = self.cell(x, y)
        cells = int(reach / self.resolution) + 1
        best, best_distance = None, math.inf

        for di in range(-cells, cells + 1):
            for dj in range(-cells, cells + 1):
                cell = (centre[0] + di, centre[1] + dj)

                if not self.inside(cell) or not self.free[cell]:
                    continue

                if not math.isnan(level) and abs(
                        self.height[cell] - level) > STEP:
                    continue

                cx, cy = self.centre(cell)
                distance = math.hypot(cx - x, cy - y)

                if distance < best_distance:
                    best, best_distance = cell, distance

        return best

    def plan(self, start, goal):
        """Cheapest chain of cells from start to goal, or None."""
        r = self.resolution
        goal_x, goal_y = self.centre(goal)

        def estimate(cell):
            x, y = self.centre(cell)
            return math.hypot(goal_x - x, goal_y - y)

        cost = {start: 0.0}
        previous = {}
        frontier = [(estimate(start), start)]
        closed = set()

        while frontier:
            _, here = heapq.heappop(frontier)

            if here == goal:
                path = [here]
                while here in previous:
                    here = previous[here]
                    path.append(here)
                return path[::-1]

            if here in closed:
                continue
            closed.add(here)

            for di, dj in MOVES:
                there = (here[0] + di, here[1] + dj)

                if (there in closed or not self.inside(there)
                        or not self.free[there]
                        or not self.aligned(here, di, dj)
                        or not self.aligned(there, di, dj)):
                    continue

                step = r * math.hypot(di, dj) * 0.5 * (
                    self.cost[here] + self.cost[there])
                total = cost[here] + step

                if total < cost.get(there, math.inf):
                    cost[there] = total
                    previous[there] = here
                    heapq.heappush(frontier, (total + estimate(there), there))

        return None

    def route(self, hunter_x, hunter_y, goal_x, goal_y, target_x, target_y):
        """Path for the hunter to the target, as points from the hunter's
        position on, pulled straight wherever it can drive straight.

        goal is where the hunter means to meet the target, used if it is
        open ground at the target's height, else the target is used. A
        path of just [hunter, goal] means the straight line is clear. None
        if there is no way there, or either robot is off the map.
        """
        target_level = self.height_at(target_x, target_y)
        hunter_level = self.height_at(hunter_x, hunter_y)

        if math.isnan(target_level) or math.isnan(hunter_level):
            return None

        goal_cell = self.cell(goal_x, goal_y)
        if (not self.inside(goal_cell) or not self.free[goal_cell]
                or abs(self.height[goal_cell] - target_level) > STEP):
            goal_cell = self.nearest_free(target_x, target_y, target_level)
            if goal_cell is None:
                return None
            goal_x, goal_y = self.centre(goal_cell)

        if self.clear_line(hunter_x, hunter_y, goal_x, goal_y):
            return [(hunter_x, hunter_y), (goal_x, goal_y)]

        start_cell = self.cell(hunter_x, hunter_y)
        if not self.inside(start_cell) or not self.free[start_cell]:
            start_cell = self.nearest_free(hunter_x, hunter_y, hunter_level)
            if start_cell is None:
                return None

        cells = self.plan(start_cell, goal_cell)
        if cells is None:
            return None

        points = ([(hunter_x, hunter_y)]
                  + [self.centre(c) for c in cells[1:-1]]
                  + [(goal_x, goal_y)])

        return self.pull_straight(points)

    def pull_straight(self, points):
        """Drops the points that can be driven past in a straight line."""
        pulled = [points[0]]
        index = 0

        while index < len(points) - 1:
            ax, ay = points[index]
            reach = index + 1

            while (reach + 1 < len(points)
                   and self.clear_line(ax, ay, *points[reach + 1])):
                reach += 1

            pulled.append(points[reach])
            index = reach

        return pulled


def path_length(points):
    return sum(math.hypot(bx - ax, by - ay)
               for (ax, ay), (bx, by) in zip(points, points[1:]))


def corner_angle(a, b, c):
    """How far the path turns at b, in radians."""
    first = math.atan2(b[1] - a[1], b[0] - a[0])
    second = math.atan2(c[1] - b[1], c[0] - b[0])
    return abs(math.atan2(math.sin(second - first), math.cos(second - first)))


def next_waypoint(points, standoff, reached=0.03, sharp=math.radians(45)):
    """Where to head along a route of more than two points, as (x, y,
    distance left to drive). The hunter slows for a sharp corner, so it
    lines up at the foot of a ramp rather than overshooting it."""
    here = points[0]

    # Close enough to a corner to be turning for the next leg.
    if len(points) > 2 and math.dist(here, points[1]) < reached:
        points = [here] + points[2:]

    left = max(0.0, path_length(points) - standoff)
    waypoint = points[1]

    if len(points) > 2 and corner_angle(here, waypoint, points[2]) > sharp:
        left = min(left, math.dist(here, waypoint) + reached)

    return waypoint[0], waypoint[1], left
