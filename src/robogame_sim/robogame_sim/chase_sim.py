"""A quick 2D stand-in for Gazebo, to try Chaser against moving targets.

    python3 -m robogame_sim.chase_sim [LAYOUT ...] [--seeds N]

Both robots are unicycles with acceleration limits, a little pose noise and
a tick of latency, on a height map of the layout eight times finer than
the planner's. A move is refused when the body would run into a wall, a step
up, or put a wheel over a drop, so a robot pressed against something stays
put, as it would against a wall in Gazebo, but without sliding off it.

The target drives itself: still, wandering about the arena, circling the
hunter, running from it, or driving like a person on a joystick. Each run
reports how soon the hunter first reached the target, how much of the time
after that it stayed with it, and the longest it went without getting
anywhere while not with the target.

With the target counts the hunter's front within 6 cm of it: a target
circling the hunter as fast as it can turn stays a few centimetres off.
"""

import argparse
import math
import random
import time

import numpy as np

from robogame_sim import arena_layout
from robogame_sim.chase import Chaser, drive_command
from robogame_sim.chase import ARM_REACH
from robogame_sim.robot_sdf import SHOULDER, WHEEL_RADIUS, WHEEL_X
from robogame_sim.terrain import (
    HALF_LENGTH,
    HALF_WIDTH,
    STEP,
    Terrain,
    next_waypoint,
    path_length,
    rasterise,
)

DT = 0.01
TICK = 0.05
MAX_LINEAR = 0.5
MAX_ANGULAR = 1.5
ACCELERATION = 1.5
ANGULAR_ACCELERATION = 12.0
FINE = 8


def body_points(spacing=0.005):
    """The outline of the robot's body, and where its wheels touch."""
    xs = np.arange(-HALF_LENGTH, HALF_LENGTH + 1e-9, spacing)
    ys = np.arange(-HALF_WIDTH, HALF_WIDTH + 1e-9, spacing)
    outline = np.concatenate([
        np.stack([xs, np.full_like(xs, -HALF_WIDTH)], 1),
        np.stack([xs, np.full_like(xs, HALF_WIDTH)], 1),
        np.stack([np.full_like(ys, -HALF_LENGTH), ys], 1),
        np.stack([np.full_like(ys, HALF_LENGTH), ys], 1)])
    wheels = np.array([(sx * WHEEL_X, sy * HALF_WIDTH)
                       for sx in (-1, 1) for sy in (-1, 1)])
    return outline, wheels


class World:
    """A fine height map, and whether a robot fits at a pose on it."""

    def __init__(self, triangles, terrain):
        self.terrain = terrain
        self.resolution = terrain.resolution / FINE
        self.origin = terrain.origin
        shape = (terrain.shape[0] * FINE, terrain.shape[1] * FINE)
        self.height = rasterise(triangles, self.origin, shape,
                                self.resolution)
        self.outline, self.wheels = body_points()
        self.samples = np.linspace(0.0, 1.0, 24)
        reach = np.arange(SHOULDER[0], ARM_REACH + 1e-9, 0.005)
        self.arm = np.stack([reach, np.zeros_like(reach)], axis=1)

    def heights(self, points):
        i = np.floor((points[..., 0] - self.origin[0])
                     / self.resolution).astype(int)
        j = np.floor((points[..., 1] - self.origin[1])
                     / self.resolution).astype(int)
        inside = ((i >= 0) & (i < self.height.shape[0])
                  & (j >= 0) & (j < self.height.shape[1]))
        out = np.full(points.shape[:-1], np.nan)
        out[inside] = self.height[i[inside], j[inside]]
        return out

    def rays(self, x, y, yaw, points):
        """Heights from the centre out to each point, along a ray."""
        c, s = math.cos(yaw), math.sin(yaw)
        px = x + points[:, 0] * c - points[:, 1] * s
        py = y + points[:, 0] * s + points[:, 1] * c
        t = self.samples[None, :]
        ray = np.stack([x + (px[:, None] - x) * t,
                        y + (py[:, None] - y) * t], axis=-1)
        return self.heights(ray)

    def fits(self, x, y, yaw, arm=False):
        """Whether the robot can be at x, y, yaw: nothing it would be in
        rises a step above the ground under it, and its wheels are all on
        ground it can drive to. With arm, its arm is out level in front at
        shoulder height, and hits anything taller, like a wall."""
        body = self.rays(x, y, yaw, self.outline)
        with np.errstate(invalid="ignore"):
            if (np.diff(body, axis=1) > STEP).any():
                return False

        if arm:
            c, s = math.cos(yaw), math.sin(yaw)
            points = np.stack([x + self.arm[:, 0] * c,
                               y + self.arm[:, 0] * s], axis=1)
            shoulder = self.heights(np.array([x, y])) + (
                WHEEL_RADIUS + SHOULDER[2])
            with np.errstate(invalid="ignore"):
                if (self.heights(points) > shoulder).any():
                    return False

        wheels = self.rays(x, y, yaw, self.wheels)
        if np.isnan(wheels).any():
            return False

        return not (np.abs(np.diff(wheels, axis=1)) > STEP).any()


class Robot:
    """A unicycle that answers commands as the ideal drive model does, with
    the acceleration a light robot has."""

    def __init__(self, world, x, y, yaw, arm=False):
        self.world = world
        self.arm = arm
        self.x, self.y, self.yaw = x, y, yaw
        self.linear = self.angular = 0.0
        self.blocked = 0

    def step(self, linear, angular):
        linear = max(-MAX_LINEAR, min(linear, MAX_LINEAR))
        angular = max(-MAX_ANGULAR, min(angular, MAX_ANGULAR))
        dv = ACCELERATION * DT
        dw = ANGULAR_ACCELERATION * DT
        self.linear += max(-dv, min(linear - self.linear, dv))
        self.angular += max(-dw, min(angular - self.angular, dw))

        yaw = self.yaw + self.angular * DT
        mid = self.yaw + self.angular * DT / 2
        x = self.x + self.linear * DT * math.cos(mid)
        y = self.y + self.linear * DT * math.sin(mid)

        # Blocked, it turns or slides if either alone fits, else stays.
        for nx, ny, nyaw in ((x, y, yaw), (self.x, self.y, yaw),
                             (x, y, self.yaw)):
            if self.world.fits(nx, ny, nyaw, self.arm):
                if (nx, ny, nyaw) != (x, y, yaw):
                    self.blocked += 1
                self.x, self.y, self.yaw = nx, ny, nyaw
                return

        self.blocked += 1
        self.linear = self.angular = 0.0


class Driver:
    """The target: drives itself to goals picked by behaviour."""

    def __init__(self, behaviour, terrain, rng, speed, centre=(0.0, 0.0),
                 radius=0.3):
        self.behaviour = behaviour
        self.centre = centre
        self.radius = radius
        self.terrain = terrain
        self.rng = rng
        self.speed = speed
        self.goal = None
        self.next_goal = 0.0
        self.stick = (0.0, 0.0)

    def pick(self, now, me, hunter):
        terrain, rng = self.terrain, self.rng
        if self.behaviour in ("orbit", "circle"):
            # Round and round a spot, a quarter turn ahead of where it is:
            # orbit round where the hunter started, circle round the
            # hunter wherever it goes, so it never gets any closer.
            # Further round, or nearer in, where that is off the ground.
            cx, cy = self.centre if self.behaviour == "orbit" else (
                hunter.x, hunter.y)
            angle = math.atan2(me.y - cy, me.x - cx)
            self.goal = None
            for radius in (self.radius, self.radius * 0.7):
                for turn in (0.9, 1.3, 1.8, 2.4):
                    x = cx + radius * math.cos(angle + turn)
                    y = cy + radius * math.sin(angle + turn)
                    cell = terrain.cell(x, y)
                    if terrain.inside(cell) and terrain.free[cell]:
                        self.goal = (x, y)
                        return
            return

        if now < self.next_goal and self.goal is not None:
            return

        reach = terrain.reachable(me.x, me.y)
        if reach is None:
            self.goal = None
            return

        cx, cy = terrain.centres()
        open_ground = reach & terrain.free

        if not open_ground.any():
            self.goal = None
        elif self.behaviour == "flee":
            # The open spot near here furthest from the hunter.
            near = open_ground & (np.hypot(cx - me.x, cy - me.y) < 0.4)
            if near.any():
                score = np.where(near, np.hypot(cx - hunter.x,
                                                cy - hunter.y), -1.0)
                i, j = np.unravel_index(np.argmax(score), score.shape)
                self.goal = (float(cx[i, j]), float(cy[i, j]))
            self.next_goal = now + 1.0
        else:
            cells = np.argwhere(open_ground)
            i, j = cells[rng.randrange(len(cells))]
            self.goal = (float(cx[i, j]), float(cy[i, j]))
            self.next_goal = now + rng.uniform(4.0, 8.0)

    def command(self, now, me, hunter):
        if self.behaviour == "still":
            return 0.0, 0.0

        if self.behaviour == "joystick":
            if now >= self.next_goal:
                self.stick = (self.rng.uniform(-0.1, 1.0) * self.speed,
                              self.rng.uniform(-1.5, 1.5))
                self.next_goal = now + self.rng.uniform(0.4, 1.5)
            return self.stick

        self.pick(now, me, hunter)
        if self.goal is None:
            return 0.0, 0.0

        path = self.terrain.route(me.x, me.y, *self.goal, *self.goal, me.yaw)
        if path is None or len(path) < 2:
            return 0.0, 0.0

        if len(path) == 2:
            wx, wy = path[1]
            left = path_length(path)
        else:
            wx, wy, left = next_waypoint(path, 0.0)

        can_turn = self.terrain.can_turn(me.x, me.y)
        linear, angular, _ = drive_command(
            me.x, me.y, me.yaw, wx, wy, left,
            self.speed if can_turn else min(self.speed, 0.1), 1.5, 1.5, 3.0,
            1.0, can_turn, not self.terrain.free_line(me.x, me.y, wx, wy))
        if self.terrain.on_slope(me.x, me.y) and linear > 0.0:
            linear = max(linear, 0.12)
        if left < 0.02:
            self.next_goal = now
        return linear, angular


def random_spot(terrain, rng, cells, away=None, apart=0.35):
    for _ in range(1000):
        i, j = cells[rng.randrange(len(cells))]
        x, y = terrain.centre((int(i), int(j)))
        if away is None or math.dist((x, y), away) >= apart:
            return x, y, rng.uniform(-math.pi, math.pi)
    return x, y, 0.0


def run(world, behaviour, seed, seconds=40.0, target_speed=0.18,
        chaser_factory=None, noise=0.001, trace=None, hunter_arm=False):
    """One chase. Returns a dict of how it went. A list passed as trace
    gets the time, both poses and what the hunter was doing each tick.
    hunter_arm drives the hunter with its arm out, as it was before it
    stowed it."""
    terrain = world.terrain
    rng = random.Random(seed)
    # Both robots on open ground the hunter can drive to from its spot.
    free = np.argwhere(terrain.free)
    hx, hy, hyaw = random_spot(terrain, rng, free)
    reach = terrain.reachable(hx, hy)
    cells = np.argwhere(reach & terrain.free)
    tx, ty, tyaw = random_spot(terrain, rng, cells, (hx, hy))

    # Facing a way they fit, as spawning.py has a robot face open ground.
    hyaw = next((h for h in (hyaw + k * math.pi / 4 for k in range(8))
                 if world.fits(hx, hy, h, hunter_arm)), hyaw)
    tyaw = next((h for h in (tyaw + k * math.pi / 4 for k in range(8))
                 if world.fits(tx, ty, h)), tyaw)
    hunter = Robot(world, hx, hy, hyaw, hunter_arm)
    target = Robot(world, tx, ty, tyaw)

    chaser = (chaser_factory or Chaser)(terrain)
    driver = Driver(behaviour, terrain, rng, target_speed, (hx, hy),
                    min(0.4, max(0.25, math.dist((hx, hy), (tx, ty)))))
    command = (0.0, 0.0)
    target_command = (0.0, 0.0)
    now, next_tick = 0.0, 0.0
    first_reach = None
    engaged_ticks = after_ticks = 0
    still_since, still_at = 0.0, (hx, hy)
    worst_still = 0.0
    plan_times = []

    while now < seconds:
        if now >= next_tick:
            next_tick += TICK
            # The target's pose, as the arena camera would see it.
            chaser.track(target.x + rng.gauss(0, noise),
                         target.y + rng.gauss(0, noise), now,
                         target.yaw + rng.gauss(0, noise * 5))
            started = time.perf_counter()
            # Latency: last tick's command goes out now.
            next_command = chaser.update(
                now, hunter.x + rng.gauss(0, noise),
                hunter.y + rng.gauss(0, noise),
                hunter.yaw + rng.gauss(0, noise * 5))
            plan_times.append(time.perf_counter() - started)
            hunter_command, command = command, next_command
            target_command = driver.command(now, target, hunter)
            if trace is not None:
                trace.append((now, hunter.x, hunter.y, hunter.yaw, target.x,
                              target.y, target.yaw, chaser.status))

            engaged = chaser.engaged
            if engaged and first_reach is None:
                first_reach = now
            if first_reach is not None:
                after_ticks += 1
                engaged_ticks += chaser.gap < 0.06

            if engaged or math.dist((hunter.x, hunter.y), still_at) > 0.03:
                still_since, still_at = now, (hunter.x, hunter.y)
            worst_still = max(worst_still, now - still_since)

        hunter.step(*hunter_command)
        target.step(*target_command)
        now += DT

    return {
        "behaviour": behaviour,
        "seed": seed,
        "first_reach": first_reach,
        "with_target": engaged_ticks / after_ticks if after_ticks else 0.0,
        "worst_still": worst_still,
        "blocked": hunter.blocked,
        "plan_ms": 1000 * float(np.mean(plan_times)),
        "plan_worst_ms": 1000 * float(np.max(plan_times)),
        "reachable": bool(reach[terrain.cell(target.x, target.y)])
        if terrain.inside(terrain.cell(target.x, target.y)) else False,
        "start": (round(hx, 3), round(hy, 3), round(tx, 3), round(ty, 3)),
    }


def load_world(layout):
    """The World for a layout, by name or path, or as a dict."""
    layout = (arena_layout.normalise(layout) if isinstance(layout, dict)
              else arena_layout.load(layout))
    triangles = arena_layout.triangles(layout)
    return World(triangles, Terrain(triangles))


BEHAVIOURS = ("still", "wander", "orbit", "circle", "flee", "joystick")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("layouts", nargs="*", default=["default"])
    parser.add_argument("--seeds", type=int, default=4)
    parser.add_argument("--seconds", type=float, default=40.0)
    parser.add_argument("--behaviours", default=",".join(BEHAVIOURS))
    args = parser.parse_args(argv)

    for name in args.layouts:
        world = load_world(name)
        print(f"== {name}")
        for behaviour in args.behaviours.split(","):
            for seed in range(args.seeds):
                r = run(world, behaviour, seed, args.seconds)
                reach = (f"{r['first_reach']:5.1f}s"
                         if r["first_reach"] is not None else "never")
                print(f"{behaviour:9s} seed {seed}: reached {reach}, "
                      f"with it {100 * r['with_target']:3.0f}% after, "
                      f"longest stuck {r['worst_still']:4.1f}s, "
                      f"blocked {r['blocked']:4d}, "
                      f"plan {r['plan_ms']:.1f}/{r['plan_worst_ms']:.0f} ms"
                      + ("" if r["reachable"]
                         else " (target ended out of reach)"))


if __name__ == "__main__":
    main()
