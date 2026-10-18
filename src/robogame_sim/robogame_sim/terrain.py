"""The arena's terrain, and paths over it for chase_node. No ROS in here.

The collision geometry of the arena model is rasterised into a height map:
the top of every box, and the upward faces of every mesh. Wherever the
height jumps, as at a platform's edge, a wall, a ramp's sides or the edge of
the floor tiles, is a step the wheels cannot climb. The steps are found on
a raster FINE times finer than the cells, so each cell knows how far it is
from the nearest one to a couple of millimetres: tight gaps between walls
need that.

Where a cell has TURN_CLEARANCE the robot can turn on the spot. Where it
only has CLEARANCE, in a gap narrower than the robot is long, the robot can
still drive through lined up with the gap, along one of the four grid
axes, if its body fits there that way round. It cannot turn in such a cell,
so a path only enters one lined up and leaves it the same way.

Where the ground slopes, on a ramp, the robot may only drive up or down the
slope, and that holds for LINE_UP around it too, so it lines up before
climbing and comes off the top straight.

A path from the hunter to the target is planned with A* over the cells,
and the way the robot faces in each narrow one, then pulled straight, so
the hunter goes round a platform that is in the way, through a gap between
walls, and up a ramp to reach a target on top of one.
"""

import heapq
import math
import os
import re
import struct
import xml.etree.ElementTree as ElementTree

import numpy as np

from robogame_sim.robot_sdf import (
    CHASSIS,
    WHEEL_RADIUS,
    WHEEL_WIDTH,
    WHEEL_X,
    WHEEL_Y,
)

# Half an inch: on a layout every module edge is on this lattice, and so is
# the middle of every gap between two modules, so with cell centres on it
# a gap is planned through its middle.
RESOLUTION = 0.0127
FINE = 4
# Height change between neighbouring cells that is a step, not a slope.
# The arena ramps rise at most about 0.006 m per cell, the platforms 0.025.
STEP = 0.013
# Gradient above which the ground counts as a ramp, and from which on it is
# a step: a ramp rises at most about 0.5, a step of STEP between two cells
# about 0.5 across the two either side of it, a module's edge more.
SLOPE = 0.05
STEP_GRADIENT = 0.75

# The robot's body seen from above, wheels and all, with its arm stowed
# over the chassis: the wheels stand out past both ends of the chassis.
HALF_LENGTH = max(CHASSIS[0] / 2, WHEEL_X + WHEEL_RADIUS)
HALF_WIDTH = max(CHASSIS[1] / 2, WHEEL_Y + WHEEL_WIDTH / 2)
# Turning on the spot, the body's corners sweep a circle, which still just
# fits in a 5 inch gap between walls.
TURN_CLEARANCE = math.hypot(HALF_LENGTH, HALF_WIDTH) + 0.002
# Room either side of the body, and at its ends, to drive lined up through
# a gap it cannot turn in.
SIDE_MARGIN = 0.01
END_MARGIN = 0.004
CLEARANCE = HALF_WIDTH + SIDE_MARGIN
# A drop, the edge of the floor or of a platform the robot is on, only has
# to stay clear of where the wheels touch the ground: the ends of the body
# may hang over it.
DROP_HALF_LENGTH = WHEEL_X + 0.008
DROP_HALF_WIDTH = HALF_WIDTH + 0.005
DROP_TURN_CLEARANCE = math.hypot(WHEEL_X, HALF_WIDTH) + 0.004
# Cells closer than SOFT_CLEARANCE to a step cost more, so paths keep a
# little further off, and lined up only cells cost the most, so a gap is
# used when it saves going a long way round.
SOFT_CLEARANCE = 0.1
NARROW_COST = 3.0
LINE_UP = 0.07
# Room round the body that keeps it off a ramp it is driving across.
SLOPE_MARGIN = 0.015
# Moves on and around a ramp must be within 15 degrees of its slope.
ALIGN = math.cos(math.radians(15))
# The robot steps sideways into a lane beside it, one cell over while it
# drives LANE_CHANGE cells along, a little under 10 degrees off the lane.
# A straight line through cells the robot must be lined up in may be that
# far off their axis.
LANE_CHANGE = 6
LINE_TOLERANCE = math.radians(10)
# What backing out of a gap it is lined up in costs over driving on, in
# metres of path.
REVERSE_COST = 0.2
# A robot counts as lined up along an axis within this of it.
HEADING_FIT = math.radians(30)

# Opposite moves are four apart, and d % 4 is the axis a move runs along.
MOVES = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
AXES = [math.pi / 4 * k for k in range(4)]


class Path(list):
    """Points from the hunter on. partial is True when the path stops at
    the nearest place to a target the hunter cannot reach."""

    def __init__(self, points, partial=False):
        super().__init__(points)
        self.partial = partial


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
    """Every collision surface in an SDF model file, in the model's frame.
    Mesh paths are taken relative to the model's folder."""
    model_dir = os.path.dirname(os.path.abspath(path))

    def resolve(uri):
        return os.path.join(model_dir, re.sub(r"^model://[^/]+/", "", uri))

    return sdf_triangles(ElementTree.parse(path).getroot(), resolve)


def sdf_triangles(root, resolve):
    """Every collision surface under an SDF element, in its frame. resolve
    turns a mesh uri into a file path."""
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
                faces = read_stl(resolve(mesh.find("uri").text.strip()))
                scale = mesh.find("scale")

                if scale is not None:
                    faces = faces * np.array(
                        [float(v) for v in scale.text.split()])
            else:
                continue

            faces = faces @ local_rotation.T + local_translation
            triangles.append(faces @ link_rotation.T + link_translation)

    return np.concatenate(triangles) if triangles else np.zeros((0, 3, 3))


def rasterise(triangles, origin, shape, resolution):
    """Highest upward facing surface over each cell centre of a grid, NaN
    where there is none."""
    height = np.full(shape, np.nan)
    r = resolution
    xs = origin[0] + (np.arange(shape[0]) + 0.5) * r
    ys = origin[1] + (np.arange(shape[1]) + 0.5) * r

    for a, b, c in triangles:
        normal = np.cross(b - a, c - a)
        normal_z = normal[2]
        area = np.linalg.norm(normal)

        # Walls are not ground. Undersides are kept: they are under
        # the top, so the highest surface still wins.
        if area == 0 or abs(normal_z) < 0.3 * area:
            continue

        lo = np.floor((np.minimum(np.minimum(a, b), c)[:2]
                       - origin) / r - 0.5).astype(int)
        hi = np.ceil((np.maximum(np.maximum(a, b), c)[:2]
                      - origin) / r - 0.5).astype(int)
        lo = np.clip(lo, 0, None)
        hi = np.minimum(hi, np.array(shape) - 1)

        if (hi < lo).any():
            continue

        x, y = np.meshgrid(xs[lo[0]:hi[0] + 1], ys[lo[1]:hi[1] + 1],
                           indexing="ij")

        # Barycentric coordinates of each cell centre.
        u = ((b[1] - c[1]) * (x - c[0])
             + (c[0] - b[0]) * (y - c[1])) / normal_z
        v = ((c[1] - a[1]) * (x - c[0])
             + (a[0] - c[0]) * (y - c[1])) / normal_z
        w = 1 - u - v
        # A cell centre on the join between two modules is on both, though
        # their meshes' float32 edges miss it by a little.
        inside = (u >= -1e-6) & (v >= -1e-6) & (w >= -1e-6)
        z = u * a[2] + v * b[2] + w * c[2]

        patch = height[lo[0]:hi[0] + 1, lo[1]:hi[1] + 1]
        patch[inside] = np.fmax(patch[inside], z[inside])

    return height


def step_edges(height, origin, resolution):
    """Points half way between neighbouring cells of a height map that a
    step lies between, the edge of the map included, as rows of x, y and
    the ground height below and above the step, -inf below for none."""
    padded = np.pad(height, 1, constant_values=np.nan)
    xs = origin[0] + (np.arange(-1, height.shape[0] + 1) + 0.5) * resolution
    ys = origin[1] + (np.arange(-1, height.shape[1] + 1) + 0.5) * resolution
    rows = []

    for a, b, di, dj in ((padded[:-1], padded[1:], 1, 0),
                         (padded[:, :-1], padded[:, 1:], 0, 1)):
        with np.errstate(invalid="ignore"):
            edge = (np.isnan(a) != np.isnan(b)) | (np.abs(a - b) > STEP)

        i, j = np.nonzero(edge)
        ha = np.nan_to_num(a[i, j], nan=-np.inf)
        hb = np.nan_to_num(b[i, j], nan=-np.inf)
        rows.append(np.stack([(xs[i] + xs[i + di]) / 2,
                              (ys[j] + ys[j + dj]) / 2,
                              np.minimum(ha, hb), np.maximum(ha, hb)],
                             axis=1))

    return np.concatenate(rows)


def shifted(array, di, dj, fill):
    """array moved so that out[i, j] is array[i + di, j + dj]."""
    out = np.full_like(array, fill)
    w, h = array.shape[:2]
    out[max(0, -di):w - max(0, di), max(0, -dj):h - max(0, dj)] = \
        array[max(0, di):w - max(0, -di), max(0, dj):h - max(0, -dj)]
    return out


def line_axis(dx, dy):
    """The grid axis, 0 to 3, a line along dx, dy runs along to within
    LINE_TOLERANCE, or None."""
    angle = math.atan2(dy, dx) % math.pi
    axis = int(round(angle / (math.pi / 4)))
    off = abs(angle - axis * math.pi / 4)

    return axis % 4 if off <= LINE_TOLERANCE else None


class Terrain:
    """Height map of the arena, with the cells a robot can drive on, and
    the way it must face in those it cannot turn in."""

    def __init__(self, triangles, resolution=RESOLUTION):
        self.resolution = r = resolution
        corners = triangles.reshape(-1, 3)[:, :2]
        # Cell centres on multiples of the resolution.
        first = np.floor(corners.min(axis=0) / r + 0.5)
        last = np.ceil(corners.max(axis=0) / r - 0.5)
        self.origin = (first - 0.5) * r
        self.shape = (int(last[0] - first[0]) + 1,
                      int(last[1] - first[1]) + 1)
        self.xs = self.origin[0] + (np.arange(self.shape[0]) + 0.5) * r
        self.ys = self.origin[1] + (np.arange(self.shape[1]) + 0.5) * r

        self.height = rasterise(triangles, self.origin, self.shape, r)
        fine = rasterise(triangles, self.origin,
                         (self.shape[0] * FINE, self.shape[1] * FINE),
                         r / FINE)
        edges = step_edges(fine, self.origin, r / FINE)

        self.find_clearance(edges)
        self.find_slopes()
        self.find_room(edges)
        self.build_graph()
        self.index_edges(edges)

    @classmethod
    def from_sdf(cls, path, resolution=RESOLUTION):
        return cls(model_triangles(path), resolution)

    # -- building the map -------------------------------------------------

    def centres(self):
        return np.meshgrid(self.xs, self.ys, indexing="ij")

    def near_edges(self, edges, reach):
        """For every offset within reach cells, the cell each step edge
        point lands in moved by it, those inside the map, the point's
        offset from that cell's centre, and whether the step is a drop
        seen from that cell: whether the cell is on its high side."""
        r = self.resolution
        ci = np.floor((edges[:, 0] - self.origin[0]) / r).astype(int)
        cj = np.floor((edges[:, 1] - self.origin[1]) / r).astype(int)
        level = np.nan_to_num(self.height, nan=-np.inf)

        for di in range(-reach, reach + 1):
            for dj in range(-reach, reach + 1):
                i, j = ci + di, cj + dj
                ok = ((i >= 0) & (i < self.shape[0])
                      & (j >= 0) & (j < self.shape[1]))
                i, j, near = i[ok], j[ok], edges[ok]
                h = level[i, j]

                with np.errstate(invalid="ignore"):
                    drop = h - near[:, 2] > near[:, 3] - h

                yield (i, j, near[:, 0] - self.xs[i],
                       near[:, 1] - self.ys[j], drop)

    def find_clearance(self, edges):
        """Distance from each cell centre to the nearest wall, a step up,
        and to the nearest drop, as far as SOFT_CLEARANCE, inf beyond.

        clearance is the two in one, a drop counting as a wall that much
        further off as the wheels sit in from the body's corners, so a
        cell is free, with room to turn, where clearance is at least
        TURN_CLEARANCE.
        """
        wall = np.full(self.shape, np.inf)
        drop = np.full(self.shape, np.inf)
        reach = int(math.ceil(SOFT_CLEARANCE / self.resolution)) + 1

        for i, j, dx, dy, is_drop in self.near_edges(edges, reach):
            distance = np.hypot(dx, dy)
            np.minimum.at(wall, (i[~is_drop], j[~is_drop]),
                          distance[~is_drop])
            np.minimum.at(drop, (i[is_drop], j[is_drop]), distance[is_drop])

        ground = ~np.isnan(self.height)
        self.clearance = np.where(ground, np.minimum(
            wall, drop + TURN_CLEARANCE - DROP_TURN_CLEARANCE), 0.0)
        self.room = (ground & (wall >= CLEARANCE)
                     & (drop >= DROP_HALF_WIDTH))
        self.free = self.clearance >= TURN_CLEARANCE
        self.cost = 1.0 + 2.0 * np.clip(
            (SOFT_CLEARANCE - self.clearance)
            / (SOFT_CLEARANCE - TURN_CLEARANCE), 0.0, 1.0)

    def find_room(self, edges):
        """lined_up[a]: the robot's body fits in the cell facing along
        axis a, either way, though it may not be able to turn there."""
        sizes = ((HALF_LENGTH + END_MARGIN, HALF_WIDTH + SIDE_MARGIN),
                 (DROP_HALF_LENGTH, DROP_HALF_WIDTH))
        blocked = np.zeros((4,) + self.shape, dtype=bool)
        reach = int(math.ceil(math.hypot(*sizes[0]) / self.resolution)) + 1

        for i, j, dx, dy, is_drop in self.near_edges(edges, reach):
            half_length = np.where(is_drop, sizes[1][0], sizes[0][0])
            half_width = np.where(is_drop, sizes[1][1], sizes[0][1])

            for a, angle in enumerate(AXES):
                c, s = math.cos(angle), math.sin(angle)
                inside = ((np.abs(dx * c + dy * s) < half_length)
                          & (np.abs(dy * c - dx * s) < half_width))
                blocked[a, i[inside], j[inside]] = True

        self.lined_up = self.room & ~blocked
        self.cost[~self.free] = NARROW_COST

    def find_slopes(self):
        """The slope directions that moves on and around a ramp must follow,
        as unit vectors, zero where moves are free.

        axes holds one for each ramp near a cell, told apart by the grid
        axis its slope runs along, and a move may follow any of them: on a
        one module landing where a ramp comes up from one side and the next
        goes on up from another, the robot drives along either, as it comes
        off one and lines up for the other. Averaged, they would only allow
        the diagonal between them. On a ramp itself only its own slope is
        kept. axis is the strongest of them."""
        h = np.where(np.isnan(self.height), 0.0, self.height)
        gx, gy = np.gradient(h, self.resolution)
        magnitude = np.hypot(gx, gy)
        sloped = self.room & (magnitude > SLOPE)
        self.sloped = (magnitude > SLOPE) & ~np.isnan(self.height)

        cx, cy = self.centres()
        self.axes = np.zeros(self.shape + (len(AXES), 2))
        self.axis = np.zeros(self.shape + (2,))
        self.over_slope = np.zeros((len(AXES),) + self.shape, dtype=bool)

        if not sloped.any():
            return

        sx, sy = cx[sloped], cy[sloped]
        ux, uy = gx[sloped] / magnitude[sloped], gy[sloped] / magnitude[sloped]
        # Which grid axis each slope runs along, either way up it, turned
        # to point the same way along it, so ramps facing each other add up
        # rather than cancel.
        bins = np.rint((np.arctan2(uy, ux) % math.pi)
                       / (math.pi / 4)).astype(int) % len(AXES)
        flip = (ux * np.cos(np.array(AXES))[bins]
                + uy * np.sin(np.array(AXES))[bins]) < 0
        ux, uy = np.where(flip, -ux, ux), np.where(flip, -uy, uy)
        flat = np.stack([cx.ravel(), cy.ravel()], axis=1)
        sums = np.zeros((len(flat), len(AXES), 2))

        for start in range(0, len(sx), 256):
            part = slice(start, start + 256)
            near = np.hypot(flat[:, None, 0] - sx[None, part],
                            flat[:, None, 1] - sy[None, part]) <= LINE_UP
            for b in range(len(AXES)):
                mask = bins[part] == b
                if mask.any():
                    sums[:, b, 0] += near[:, mask] @ ux[part][mask]
                    sums[:, b, 1] += near[:, mask] @ uy[part][mask]

        norm = np.hypot(sums[..., 0], sums[..., 1])
        # A ramp counts where it is a good part of the slope round about, so
        # a few odd cells at a ramp's side do not open up other directions.
        keep = (norm > 1e-9) & (norm >= 0.3 * norm.max(axis=1, keepdims=True))
        own = np.full(len(flat), -1)
        own[np.flatnonzero(sloped.ravel())] = bins
        on_ramp = own >= 0
        keep[on_ramp] = False
        keep[on_ramp, own[on_ramp]] = norm[on_ramp, own[on_ramp]] > 1e-9

        axes = np.zeros_like(sums)
        axes[keep] = sums[keep] / norm[keep][:, None]
        strongest = np.argmax(np.where(keep, norm, -1.0), axis=1)
        axis = axes[np.arange(len(flat)), strongest]
        self.axes = axes.reshape(self.shape + (len(AXES), 2))
        self.axis = axis.reshape(self.shape + (2,))

        # A robot driving across a slope only reaches its half width toward
        # it, not the LINE_UP its front does, so off the top of a ramp it
        # may turn and drive along the ramp's top edge once its body is
        # clear of the slope. over_slope[a]: its body, facing along axis a,
        # is over a ramp. That is all of the ramp, its sides too, which have
        # no room to be on, but not the steps round it, which are steeper.
        ramp = self.sloped & (magnitude < STEP_GRADIENT)
        half_length = HALF_LENGTH + SLOPE_MARGIN
        half_width = HALF_WIDTH + SLOPE_MARGIN
        reach = int(math.ceil(math.hypot(half_length, half_width)
                              / self.resolution))

        for di in range(-reach, reach + 1):
            for dj in range(-reach, reach + 1):
                moved = shifted(ramp, di, dj, False)
                ox, oy = di * self.resolution, dj * self.resolution
                for a, angle in enumerate(AXES):
                    c, s = math.cos(angle), math.sin(angle)
                    if (abs(ox * c + oy * s) <= half_length
                            and abs(oy * c - ox * s) <= half_width):
                        self.over_slope[a] |= moved

    def along_slope(self, i, j, dx, dy):
        """Whether moving along dx, dy is allowed by the ramps in each of
        cells i, j, as a boolean array: there is no slope round about, the
        move follows one of them, or the robot's body is clear of them."""
        axes = self.axes[i, j]
        level = ~axes.any(axis=(-2, -1))
        length = np.hypot(dx, dy)
        dot = np.abs(axes[..., 0] * dx + axes[..., 1] * dy)

        # Facing between two grid axes, it may be over a slope either way.
        angle = math.atan2(dy, dx) % math.pi / (math.pi / 4)
        nearest = int(round(angle)) % len(AXES)
        over = self.over_slope[nearest, i, j]
        if abs(angle - round(angle)) * math.pi / 4 > LINE_TOLERANCE:
            over = over | self.over_slope[int(math.floor(angle)) % len(AXES),
                                        i, j] | self.over_slope[
                int(math.ceil(angle)) % len(AXES), i, j]

        return level | ~over | (dot >= ALIGN * length).any(axis=-1)

    def build_graph(self):
        """The states A* plans over and the moves between them.

        A state is a cell and, in a cell the robot cannot turn in, the axis
        it faces along there. Moves are symmetric, so a path back is the
        same path reversed, and which states can reach which is worked out
        here once.
        """
        # kinds[0] is turning on the spot, kinds[1 + a] lined up on axis a.
        kinds = np.zeros((5,) + self.shape, dtype=bool)
        kinds[0] = self.free
        kinds[1:] = self.lined_up & ~self.free
        self.state_id = np.full(kinds.shape, -1)
        self.state_id[kinds] = np.arange(int(kinds.sum()))
        self.state_kind, self.state_i, self.state_j = np.nonzero(kinds)
        count = len(self.state_i)

        passable = self.free | self.lined_up
        level = ~self.axes.any(axis=(-2, -1))
        every_i, every_j = np.indices(self.shape)
        edges = [[] for _ in range(count)]

        for d, (di, dj) in enumerate(MOVES):
            a = d % 4
            length = math.hypot(di, dj)
            # The ramp rule for moving along di, dj, in a cell.
            along = self.along_slope(every_i, every_j, di, dj)
            there_ok = shifted(passable[a] & along, di, dj, False)
            ci, cj = np.nonzero(passable[a] & along & there_ok)
            ni, nj = ci + di, cj + dj

            here = np.where(self.free[ci, cj], self.state_id[0, ci, cj],
                            self.state_id[1 + a, ci, cj])
            there = np.where(self.free[ni, nj], self.state_id[0, ni, nj],
                             self.state_id[1 + a, ni, nj])
            step = (self.resolution * length * 0.5
                    * (self.cost[ci, cj] + self.cost[ni, nj]))

            for s, t, c in zip(here.tolist(), there.tolist(), step.tolist()):
                edges[s].append((t, c))

        # Lane changes, on level ground along the x and y axes: both lanes
        # must have room lined up all the way along.
        m = LANE_CHANGE
        step = self.resolution * math.hypot(m, 1) * NARROW_COST

        for a, (fi, fj) in ((0, (1, 0)), (2, (0, 1))):
            lane = passable[a] & level

            for sf in (1, -1):
                for sp in (1, -1):
                    di, dj = sf * fi, sf * fj
                    pi, pj = sp * fj, sp * fi
                    ok = np.ones(self.shape, dtype=bool)
                    for k in range(m + 1):
                        ok &= shifted(lane, k * di, k * dj, False)
                        ok &= shifted(lane, pi + k * di, pj + k * dj, False)
                    ni_off, nj_off = pi + m * di, pj + m * dj
                    ok &= ~(self.free & shifted(self.free, ni_off, nj_off,
                                                False))
                    ci, cj = np.nonzero(ok)
                    ni, nj = ci + ni_off, cj + nj_off

                    here = np.where(self.free[ci, cj],
                                    self.state_id[0, ci, cj],
                                    self.state_id[1 + a, ci, cj])
                    there = np.where(self.free[ni, nj],
                                     self.state_id[0, ni, nj],
                                     self.state_id[1 + a, ni, nj])

                    for s, t in zip(here.tolist(), there.tolist()):
                        edges[s].append((t, step))

        self.edges = edges
        self.state_x = self.xs[self.state_i].tolist()
        self.state_y = self.ys[self.state_j].tolist()

        # Which states can reach which, by flood fill.
        label = [-1] * count
        for seed in range(count):
            if label[seed] >= 0:
                continue
            label[seed] = seed
            frontier = [seed]
            while frontier:
                for t, _ in edges[frontier.pop()]:
                    if label[t] < 0:
                        label[t] = seed
                        frontier.append(t)
        self.label = np.array(label)
        self.label_size = np.bincount(self.label, minlength=count)

    def index_edges(self, edges):
        """Keeps the step edge points sorted by the cell they are in, so
        clearance_at can find those near any point."""
        r = self.resolution
        ci = np.floor((edges[:, 0] - self.origin[0]) / r).astype(int)
        cj = np.floor((edges[:, 1] - self.origin[1]) / r).astype(int)
        key = ci * self.shape[1] + cj
        order = np.argsort(key, kind="stable")
        self.edge_key = key[order]
        self.edge_rows = edges[order]

    # -- queries ----------------------------------------------------------

    def room_at(self, x, y):
        """Distance from x, y itself, not its cell's centre, to the nearest
        wall and the nearest drop, as far as TURN_CLEARANCE and inf
        beyond."""
        ci, cj = self.cell(x, y)
        reach = int(math.ceil(TURN_CLEARANCE / self.resolution)) + 1
        width = self.shape[1]
        parts = []

        for i in range(max(0, ci - reach), min(self.shape[0], ci + reach + 1)):
            lo = np.searchsorted(self.edge_key, i * width + max(0, cj - reach))
            hi = np.searchsorted(self.edge_key,
                                 i * width + min(width - 1, cj + reach),
                                 side="right")
            if hi > lo:
                parts.append(self.edge_rows[lo:hi])

        if not parts:
            return math.inf, math.inf

        level = self.height_at(x, y)
        if math.isnan(level):
            # Off the ground altogether.
            return 0.0, 0.0

        near = np.concatenate(parts)
        distance = np.hypot(near[:, 0] - x, near[:, 1] - y)
        drop = level - near[:, 2] > near[:, 3] - level
        wall = distance[~drop].min() if (~drop).any() else math.inf
        fall = distance[drop].min() if drop.any() else math.inf
        return float(wall), float(fall)

    def cell(self, x, y):
        i = int(math.floor((x - self.origin[0]) / self.resolution))
        j = int(math.floor((y - self.origin[1]) / self.resolution))
        return i, j

    def centre(self, cell):
        i, j = cell
        return (float(self.xs[0] + i * self.resolution),
                float(self.ys[0] + j * self.resolution))

    def inside(self, cell):
        return 0 <= cell[0] < self.shape[0] and 0 <= cell[1] < self.shape[1]

    def on_slope(self, x, y):
        cell = self.cell(x, y)
        return self.inside(cell) and bool(self.sloped[cell])

    def height_at(self, x, y):
        """Ground height at x, y: between the four nearest cell centres
        where they are one smooth surface, as on a ramp, else its cell's."""
        cell = self.cell(x, y)

        if not self.inside(cell):
            return math.nan

        fx = (x - self.xs[0]) / self.resolution
        fy = (y - self.ys[0]) / self.resolution
        i, j = int(math.floor(fx)), int(math.floor(fy))

        if 0 <= i < self.shape[0] - 1 and 0 <= j < self.shape[1] - 1:
            corners = self.height[i:i + 2, j:j + 2]
            if (not np.isnan(corners).any()
                    and corners.max() - corners.min() <= STEP):
                tx, ty = fx - i, fy - j
                return float(
                    corners[0, 0] * (1 - tx) * (1 - ty)
                    + corners[1, 0] * tx * (1 - ty)
                    + corners[0, 1] * (1 - tx) * ty
                    + corners[1, 1] * tx * ty)

        return self.height[cell]

    def clearance_at(self, x, y):
        """clearance at x, y itself, as far as TURN_CLEARANCE, else that of
        its cell."""
        cell = self.cell(x, y)

        if not self.inside(cell) or math.isnan(self.height[cell]):
            return 0.0

        wall, drop = self.room_at(x, y)
        exact = min(wall, drop + TURN_CLEARANCE - DROP_TURN_CLEARANCE)
        return exact if exact < TURN_CLEARANCE else float(
            max(self.clearance[cell], TURN_CLEARANCE))

    def can_turn(self, x, y):
        """Whether a robot at x, y has room to turn on the spot."""
        return self.clearance_at(x, y) >= TURN_CLEARANCE

    def passable(self, cell):
        return self.inside(cell) and bool(
            self.free[cell] or self.lined_up[(slice(None),) + cell].any())

    def aligned(self, cell, dx, dy):
        """Whether moving along dx, dy is allowed in cell by the ramps."""
        return math.hypot(dx, dy) < 1e-12 or bool(
            self.along_slope(*cell, dx, dy))

    def line_cells(self, ax, ay, bx, by):
        """The cells under a line from a to b, every half cell along it,
        or None if it leaves the map."""
        dx, dy = bx - ax, by - ay
        samples = max(1, int(math.hypot(dx, dy) / (self.resolution / 2)))
        t = np.arange(samples + 1) / samples
        i = np.floor((ax + dx * t - self.origin[0])
                     / self.resolution).astype(int)
        j = np.floor((ay + dy * t - self.origin[1])
                     / self.resolution).astype(int)

        if (i.min() < 0 or j.min() < 0 or i.max() >= self.shape[0]
                or j.max() >= self.shape[1]):
            return None

        return i, j

    def free_line(self, ax, ay, bx, by):
        """Whether a robot has room to turn all the way from a to b."""
        cells = self.line_cells(ax, ay, bx, by)
        return cells is not None and bool(self.free[cells].all())

    def slope_line(self, ax, ay, bx, by):
        """Whether a line from a to b goes over a ramp."""
        cells = self.line_cells(ax, ay, bx, by)
        return cells is not None and bool(self.sloped[cells].any())

    def clear_line(self, ax, ay, bx, by):
        """Whether a robot can drive straight from a to b: through cells it
        can turn in, or lined up with the line in those it cannot, and
        along the slope on and around a ramp."""
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        cells = self.line_cells(ax, ay, bx, by)

        if cells is None:
            return False

        i, j = cells

        if not self.free[i, j].all():
            a = line_axis(dx, dy)
            if a is None or not (self.free[i, j]
                                 | self.lined_up[a, i, j]).all():
                return False

        return length < 1e-12 or bool(
            self.along_slope(i, j, dx, dy).all())

    def open_line(self, ax, ay, bx, by, margin=0.015):
        """clear_line, keeping margin more than room to turn from steps
        along the way, or as far as either end is if that is less."""
        cells = self.line_cells(ax, ay, bx, by)

        if cells is None or not self.clear_line(ax, ay, bx, by):
            return False

        room = self.clearance[cells]
        return room.min() >= min(room[0], room[-1], TURN_CLEARANCE + margin)

    def start_states(self, cell, heading=None):
        """The states a robot in cell may be in: the one it is lined up
        nearest to, when it cannot turn there and its heading is known."""
        ids = [int(s) for s in self.state_id[(slice(None),) + cell] if s >= 0]

        if heading is None or len(ids) < 2:
            return ids

        def off(state):
            angle = AXES[self.state_kind[state] - 1]
            return abs(math.sin(heading - angle))

        return [min(ids, key=off)]

    def nearest_free(self, x, y, level, reach=0.15):
        """The cell nearest x, y the robot can be in, on ground within STEP
        of level."""
        cells = int(reach / self.resolution) + 1
        ci, cj = self.cell(x, y)
        i0, i1 = max(0, ci - cells), min(self.shape[0], ci + cells + 1)
        j0, j1 = max(0, cj - cells), min(self.shape[1], cj + cells + 1)

        if i0 >= i1 or j0 >= j1:
            return None

        ok = (self.state_id[:, i0:i1, j0:j1] >= 0).any(axis=0)
        if not math.isnan(level):
            with np.errstate(invalid="ignore"):
                ok &= np.abs(self.height[i0:i1, j0:j1] - level) <= STEP

        if not ok.any():
            return None

        x_grid, y_grid = np.meshgrid(self.xs[i0:i1], self.ys[j0:j1],
                                     indexing="ij")
        distance = np.where(ok, np.hypot(x_grid - x, y_grid - y), np.inf)
        i, j = np.unravel_index(np.argmin(distance), distance.shape)
        return int(i0 + i), int(j0 + j)

    def search(self, starts, goal, heading=None):
        """Cheapest chain of states from any of starts to a state in the
        goal cell, as cells, or None.

        With heading, the way the robot faces, backing out of a start state
        it cannot turn in, or anywhere along the gap ahead of it, costs
        REVERSE_COST more, so lined up
        in a gap it keeps on the way it is going rather than switching
        between driving on and backing out as each looks a little shorter.
        """
        # Only goals the starts connect to, so a search for one they do not
        # fails at once rather than after trying everywhere.
        labels = {int(self.label[s]) for s in starts}
        goals = {int(s) for s in self.state_id[(slice(None),) + goal]
                 if s >= 0 and int(self.label[s]) in labels}
        if not goals:
            return None

        gx, gy = self.centre(goal)
        xs, ys, edges = self.state_x, self.state_y, self.edges
        hypot, push, pop = math.hypot, heapq.heappush, heapq.heappop

        cost = {s: 0.0 for s in starts}
        previous = {}
        frontier = [(hypot(gx - xs[s], gy - ys[s]), s) for s in starts]
        heapq.heapify(frontier)
        closed = set()
        lined = set()
        if heading is not None:
            hx, hy = math.cos(heading), math.sin(heading)
            lined = {s for s in starts if self.state_kind[s] > 0}

        while frontier:
            _, here = pop(frontier)

            if here in goals:
                states = [here]
                while here in previous:
                    here = previous[here]
                    states.append(here)
                return [(int(self.state_i[s]), int(self.state_j[s]))
                        for s in reversed(states)]

            if here in closed:
                continue
            closed.add(here)
            base = cost[here]

            for there, step in edges[here]:
                total = base + step
                if here in lined:
                    # Anywhere along the gap ahead, not just where it is.
                    ahead = ((xs[there] - xs[here]) * hx
                             + (ys[there] - ys[here]) * hy)
                    if ahead < 0:
                        total += REVERSE_COST
                    elif ahead > 0 and self.state_kind[there] > 0:
                        lined.add(there)
                if total < cost.get(there, math.inf):
                    cost[there] = total
                    previous[there] = here
                    push(frontier,
                         (total + hypot(gx - xs[there], gy - ys[there]),
                          there))

        return None

    def plan(self, start, goal, heading=None):
        """Cheapest chain of cells from start to goal, or None."""
        return self.search(self.start_states(start, heading), goal)

    def route(self, hunter_x, hunter_y, goal_x, goal_y, target_x, target_y,
              hunter_yaw=None):
        """Path for the hunter to the target, as points from the hunter's
        position on, pulled straight wherever it can drive straight.

        goal is where the hunter means to meet the target, used if it is
        ground the hunter can be on at the target's height, else the target
        is used. A path of just [hunter, goal] means the straight line is
        clear. hunter_yaw, when given, says which way the hunter is lined
        up in a gap it cannot turn in.

        If the target is somewhere the hunter cannot get to, such as on a
        platform no ramp leads to, the path goes to the nearest place it
        can reach instead and its partial is True. None if either robot is
        off the map, or the hunter is boxed in.
        """
        target_level = self.height_at(target_x, target_y)
        hunter_level = self.height_at(hunter_x, hunter_y)

        if math.isnan(target_level) or math.isnan(hunter_level):
            return None

        goal_cell = self.cell(goal_x, goal_y)
        if (not self.passable(goal_cell)
                or abs(self.height[goal_cell] - target_level) > STEP):
            goal_cell = self.nearest_free(target_x, target_y, target_level)
            if goal_cell is not None:
                goal_x, goal_y = self.centre(goal_cell)

        goals = [] if goal_cell is None else [
            int(s) for s in self.state_id[(slice(None),) + goal_cell]
            if s >= 0]
        start = self.start_for(hunter_x, hunter_y, hunter_yaw, hunter_level,
                               goals)
        if start is None:
            return None
        starts = start[1]

        if goal_cell is not None:
            if self.open_line(hunter_x, hunter_y, goal_x, goal_y):
                return Path([(hunter_x, hunter_y), (goal_x, goal_y)])

            cells = self.search(starts, goal_cell, hunter_yaw)
            if cells is not None:
                return self.path_along(hunter_x, hunter_y, cells,
                                       (goal_x, goal_y))

        return self.closest_approach(
            hunter_x, hunter_y, starts, target_x, target_y, hunter_yaw)

    def start_for(self, x, y, heading, level, goals=(), reach=0.08):
        """The cell and states to plan from for a robot at x, y: its own,
        lined up as it is, if they lead to any of the states goals; else
        the nearest within reach that do, so a robot that has ended up
        somewhere awkward heads there first. With none, the nearest that
        lead to about the biggest stretch of ground round about. None if
        there is no ground it can be on near x, y."""
        here = self.cell(x, y)
        candidates = []

        if self.passable(here):
            candidates.append((here, self.start_states(here, heading)))
            candidates.append((here, self.start_states(here)))

        cells = int(reach / self.resolution) + 1
        i0 = max(0, here[0] - cells)
        i1 = min(self.shape[0], here[0] + cells + 1)
        j0 = max(0, here[1] - cells)
        j1 = min(self.shape[1], here[1] + cells + 1)

        if i0 < i1 and j0 < j1:
            ok = (self.state_id[:, i0:i1, j0:j1] >= 0).any(axis=0)
            if not math.isnan(level):
                with np.errstate(invalid="ignore"):
                    ok &= np.abs(self.height[i0:i1, j0:j1] - level) <= STEP
            ii, jj = np.nonzero(ok)
            dx, dy = self.xs[i0 + ii] - x, self.ys[j0 + jj] - y
            distance = np.hypot(dx, dy)
            # Rather one ahead or beside it than behind, so it is not sent
            # backing into the corner it has just come round.
            order = distance
            if heading is not None:
                ahead = dx * math.cos(heading) + dy * math.sin(heading)
                behind = ahead < -0.005
                order = distance + 0.05 * behind
            for k in np.argsort(order):
                if distance[k] <= reach:
                    cell = (int(i0 + ii[k]), int(j0 + jj[k]))
                    candidates.append((cell, self.start_states(cell)))

        if not candidates:
            return None

        def facing(state):
            # Free, or lined up within HEADING_FIT of the way it faces.
            kind = self.state_kind[state]
            return heading is None or kind == 0 or abs(math.sin(
                heading - AXES[kind - 1])) <= math.sin(HEADING_FIT)

        # A robot a few millimetres off a diagonal gap is in a cell only
        # lined up along the grid: rather the gap's cell beside it, lined
        # up as it faces, than plan as if it faced along the grid.
        wanted = {int(self.label[s]) for s in goals}
        for fit in (facing, lambda state: True):
            for cell, states in candidates:
                leading = [s for s in states
                           if int(self.label[s]) in wanted and fit(s)]
                if leading:
                    return cell, leading

        def size(candidate):
            return max(self.label_size[self.label[s]] for s in candidate[1])

        biggest = max(size(c) for c in candidates)
        return next(c for c in candidates if size(c) >= biggest / 2)

    def reachable_from(self, starts):
        """Which cells a robot in any of the states starts can drive to."""
        seen = np.zeros(self.shape, dtype=bool)
        mask = np.isin(self.label, self.label[list(starts)])
        seen[self.state_i[mask], self.state_j[mask]] = True
        return seen

    def reachable(self, x, y):
        """Which cells a robot starting at x, y can drive to, as a boolean
        array, or None if x, y is not ground it can be on."""
        start = self.cell(x, y)

        if not self.passable(start):
            return None

        return self.reachable_from(self.start_states(start))

    def closest_approach(self, hunter_x, hunter_y, starts, x, y,
                         heading=None):
        """Path to the place nearest x, y that the hunter can drive to."""
        if not starts:
            return None

        seen = self.reachable_from(starts)
        cx, cy = self.centres()
        # Somewhere it can turn, to wait in, if there is anywhere.
        if (seen & self.free).any():
            seen &= self.free
        distance = np.where(seen, np.hypot(cx - x, cy - y), np.inf)
        best = tuple(int(n) for n in
                     np.unravel_index(np.argmin(distance), self.shape))
        end = self.centre(best)

        if self.open_line(hunter_x, hunter_y, *end):
            return Path([(hunter_x, hunter_y), end], partial=True)

        cells = self.search(starts, best, heading)
        if cells is None:
            return None

        return self.path_along(hunter_x, hunter_y, cells, end, partial=True)

    def path_along(self, hunter_x, hunter_y, cells, end, partial=False):
        # A hunter that has been pushed off the cells it can be in heads
        # back to the one the path starts from first, unless it is about
        # there anyway.
        first = 1 if (self.cell(hunter_x, hunter_y) == cells[0]
                      or math.dist((hunter_x, hunter_y),
                                   self.centre(cells[0])) < 0.025) else 0
        points = ([(hunter_x, hunter_y)]
                  + [self.centre(c) for c in cells[first:-1]] + [end])
        return Path(self.pull_straight(points), partial)

    def pull_straight(self, points):
        """Drops the points that can be driven past in a straight line that
        keeps as far from steps as the points it replaces did, so a
        shortcut does not shave a corner the planned path stayed off."""
        room = [self.clearance[cell] if self.inside(cell) else 0.0
                for cell in (self.cell(*p) for p in points)]

        # Only the corners of a run of cells in a line need trying, each
        # with the least room along the run to it.
        corners, least_to = [points[0]], [room[0]]
        for k in range(1, len(points)):
            least_to[-1] = min(least_to[-1], room[k])
            if k == len(points) - 1 or not math.isclose(
                    (points[k][0] - points[k - 1][0])
                    * (points[k + 1][1] - points[k][1]),
                    (points[k][1] - points[k - 1][1])
                    * (points[k + 1][0] - points[k][0]), abs_tol=1e-12):
                corners.append(points[k])
                least_to.append(room[k])
        points, room = corners, [room[0]] + least_to[:-1]

        pulled = [points[0]]
        index = 0

        while index < len(points) - 1:
            ax, ay = points[index]
            reach = index + 1
            least = min(room[index], room[reach])

            while reach + 1 < len(points):
                bx, by = points[reach + 1]
                need = min(least, room[reach + 1])
                cells = self.line_cells(ax, ay, bx, by)

                if (cells is None or self.clearance[cells].min() < need
                        or not self.clear_line(ax, ay, bx, by)):
                    break

                reach += 1
                least = need

            pulled.append(points[reach])
            index = reach

        return pulled


def path_length(points):
    return sum(math.hypot(bx - ax, by - ay)
               for (ax, ay), (bx, by) in zip(points, points[1:]))


def point_along(points, distance):
    """The point distance along a path, or its end if it is shorter."""
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        leg = math.hypot(bx - ax, by - ay)
        if distance <= leg and leg > 0.0:
            return (ax + (bx - ax) * distance / leg,
                    ay + (by - ay) * distance / leg)
        distance -= leg

    return points[-1]


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
