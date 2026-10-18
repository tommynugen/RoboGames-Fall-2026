"""One chase scenario against chase_node in a headless sim already running.

    python3 scripts/gazebo_chase_test.py SCENARIO SECONDS OUT.csv \
        [--speed 0] [--step 0.002] [--layout test4]

Robot 1 hunts robot 2, as sim.launch.py robots:=1,2 chase:=1,2 has it.
This puts robot 2's arm up, then teleports both robots to the scenario's
poses and either leaves robot 2 where it is or drives it to a goal like a
player would, with a Chaser of its own. tower_nudge* knock the hunter off
line partway up the tower's spiral. It logs both robots' true 3D poses from
Gazebo to OUT.csv, and stops once the hunter has been with robot 2 for
three seconds of sim time, or after SECONDS. gazebo_chase_suite.sh runs
these, scored by gazebo_chase_score.py.

The scenarios are for test4: a tower in the middle with a ramp spiralling
up its outside to an 8 inch top, in a box of walls with a diagonal slot at
each corner the robot only gets through lined up with it.

Exits 3 if the sim sends nothing for 25 s, so the run can be tried again.
"""

import argparse
import csv
import math
import os
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from robogame_sim import arena_layout
from robogame_sim.chase import HALF_LENGTH, STOW_ARM, Chaser, standoff_distance
from robogame_sim.spawning import SPAWN_HEIGHT

WORLD = "robogame_arena"
HOLD = 3.0
LANE = 0.1524

# name: hunter (x, y, yaw), robot 2 (x, y, yaw), robot 2's goal or None
SCENARIOS = {
    # Hunter inside the box of walls, robot 2 just outside a corner slot.
    "slot_sw": ((-0.30, -0.12, -1.57), (-0.43, -0.43, 0.0), None),
    "slot_se": ((0.12, -0.30, 0.0), (0.43, -0.43, 0.0), None),
    "slot_ne": ((0.30, 0.12, 1.57), (0.43, 0.43, 0.0), None),
    "slot_nw": ((-0.12, 0.30, 3.14), (-0.43, 0.43, 0.0), None),
    # Hunter outside a corner slot, robot 2 inside past it.
    "slot_in_ne": ((0.43, 0.43, 3.14), (0.15, 0.30, 0.0), None),
    "slot_in_sw": ((-0.43, -0.43, 0.0), (-0.30, -0.15, 0.0), None),
    # Robot 2 drives out through a corner slot, the hunter after it.
    "slot_follow": ((0.05, -0.30, 3.14), (-0.27, -0.30, 3.14),
                    (-0.42, -0.42)),
    # Robot 2 on the tower's top, the hunter on the floor.
    "tower_e": ((0.30, -0.30, 3.14), (0.05, -0.03, 3.14), None),
    "tower_w": ((-0.30, 0.0, -1.57), (0.05, -0.03, 0.0), None),
    "tower_n": ((0.0, 0.30, 0.0), (0.0, 0.0, 1.57), None),
    "tower_ring": ((-0.50, 0.0, 1.57), (0.05, -0.03, 0.0), None),
    # As tower_e and tower_n, the hunter knocked off line on the way up.
    "tower_nudge": ((0.30, -0.30, 3.14), (0.05, -0.03, 3.14), None),
    "tower_nudge2": ((0.0, 0.30, 0.0), (0.0, 0.0, 1.57), None),
    # Robot 2 drives up the spiral from its foot, the hunter after it.
    "tower_follow": ((0.33, -0.33, 1.57), (0.27, -0.152, 3.14),
                     (0.0, -0.02)),
}

# Where on the spiral to knock the hunter, how far across the lane and how
# far round toward the drop.
NUDGES = [
    (lambda h: abs(h.y + LANE) < 0.02 and 0.02 < h.x < 0.07 and h.z > 0.04,
     (0.0, 0.012), 0.2),
    (lambda h: abs(h.y - LANE) < 0.02 and -0.05 < h.x < 0.0 and h.z > 0.15,
     (0.0, -0.012), 0.2),
]


class Pose:
    x = y = z = yaw = roll = pitch = None


def roll_pitch_yaw(q):
    return (math.atan2(2 * (q.w * q.x + q.y * q.z),
                       1 - 2 * (q.x ** 2 + q.y ** 2)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            math.atan2(2 * (q.w * q.z + q.x * q.y),
                       1 - 2 * (q.y ** 2 + q.z ** 2)))


def gz_service(service, reqtype, req):
    return subprocess.run(
        ["gz", "service", "-s", f"/world/{WORLD}/{service}",
         "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
         "--timeout", "3000", "--req", req],
        capture_output=True, text=True, timeout=10).stdout.strip()


def pose_request(robot, x, y, z, yaw):
    return (f'name: "{robot}", position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')


class Scenario(Node):
    def __init__(self, args):
        super().__init__("gazebo_chase_test", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.args = args
        self.hunter_start, self.target_start, self.goal = \
            SCENARIOS[args.scenario]
        self.terrain = arena_layout.terrain(arena_layout.load(
            arena_layout.find_layout(args.layout)))
        self.nudges = (list(NUDGES) if args.scenario.startswith("tower_nudge")
                       else [])
        self.hunter, self.target = Pose(), Pose()
        self.first = self.start = self.held_since = None
        self.done = False
        self.stow = STOW_ARM
        self.player = None
        if self.goal is not None:
            # A competent player, at a player's pace, right onto its goal
            # rather than a robot length short of it.
            self.player = Chaser(self.terrain, max_linear=0.2)
            self.player.target_reach = HALF_LENGTH
            self.player.clearance = -2 * HALF_LENGTH + 0.01

        self.file = open(args.out, "w", newline="")
        self.csv = csv.writer(self.file)
        self.csv.writerow(["t", "hx", "hy", "hz", "hyaw", "hroll", "hpitch",
                           "tx", "ty", "tz", "tyaw", "gap"])
        self.cmd = self.create_publisher(Twist, "/robot_2/cmd_vel", 10)
        self.arm = self.create_publisher(String, "/robot_2/arm_command", 10)

        # Odometry is 2D, so the 3D poses come from Gazebo itself.
        from gz.msgs.pose_v_pb2 import Pose_V
        from gz.transport import Node as GzNode
        self.gz_node = GzNode()
        self.gz_node.subscribe(Pose_V, f"/world/{WORLD}/dynamic_pose/info",
                               self.on_poses)
        self.create_timer(0.05, self.tick)

    def on_poses(self, message):
        for p in message.pose:
            robot = {"robot_1": self.hunter, "robot_2": self.target}.get(
                p.name)
            if robot is not None:
                robot.roll, robot.pitch, robot.yaw = roll_pitch_yaw(
                    p.orientation)
                robot.x, robot.y, robot.z = (p.position.x, p.position.y,
                                             p.position.z)

    def place(self, robot, x, y, yaw):
        # A robot teleported with its centre right on a floor tile seam
        # hangs where it is put, so keep a little off them.
        x, y = x + 0.003, y + 0.003
        under = self.terrain.height_at(x, y)
        z = SPAWN_HEIGHT + (0.0 if math.isnan(under) else max(0.0, under))
        return gz_service("set_pose", "gz.msgs.Pose",
                          pose_request(robot, x, y, z, yaw))

    def tick(self):
        if self.target.x is None or self.hunter.x is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.stow:
            # Robot 2's arm up first, so it is not put down through a wall.
            self.arm.publish(String(data=self.stow[0]))
            self.stow = self.stow[1:]

        if self.start is None:
            if self.first is None:
                self.first = now
                print("physics:", gz_service(
                    "set_physics", "gz.msgs.Physics",
                    f"max_step_size: {self.args.step}, "
                    f"real_time_factor: {self.args.speed}"), flush=True)
            if now - self.first >= 2.5:
                print("hunter placed:", self.place("robot_1",
                                                   *self.hunter_start))
                print("robot 2 placed:", self.place("robot_2",
                                                    *self.target_start),
                      flush=True)
                self.start = now
            return

        t = now - self.start
        h, g = self.hunter, self.target
        for nudge in list(self.nudges):
            where, (dx, dy), turn = nudge
            if where(h):
                print(f"knocked at {t:.1f} s:", gz_service(
                    "set_pose", "gz.msgs.Pose", pose_request(
                        "robot_1", h.x + dx, h.y + dy, h.z + 0.004,
                        h.yaw + turn)), flush=True)
                self.nudges.remove(nudge)

        linear = angular = 0.0
        if self.player is not None and t > 1.0:
            self.player.track(*self.goal, t)
            linear, angular = self.player.update(t, g.x, g.y, g.yaw)
        message = Twist()
        message.linear.x, message.angular.z = float(linear), float(angular)
        self.cmd.publish(message)

        gap = math.dist((h.x, h.y), (g.x, g.y)) - standoff_distance(
            h.x, h.y, g.x, g.y, g.yaw, 0.03, HALF_LENGTH)
        self.csv.writerow([f"{v:.4f}" for v in (
            t, h.x, h.y, h.z, h.yaw, h.roll, h.pitch,
            g.x, g.y, g.z, g.yaw, gap)])

        # With it: its front within 6 cm, on the same level, robot 2 at
        # its goal if it has one.
        with_it = (gap < 0.06 and abs(h.z - g.z) < 0.02 and t > 1.0
                   and (self.goal is None
                        or math.dist((g.x, g.y), self.goal) < 0.16))
        self.held_since = (None if not with_it else
                           t if self.held_since is None else self.held_since)
        if t >= self.args.seconds or (self.held_since is not None
                                      and t - self.held_since >= HOLD):
            self.cmd.publish(Twist())
            self.file.close()
            self.done = True


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scenario", choices=sorted(SCENARIOS))
    parser.add_argument("seconds", type=float)
    parser.add_argument("out")
    parser.add_argument("--speed", type=float, default=0.0)
    parser.add_argument("--step", type=float, default=0.002)
    parser.add_argument("--layout", default="test4")
    args = parser.parse_args()

    rclpy.init()
    node = Scenario(args)
    began = time.time()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.start is None:
            began_sim = time.time()
            if began_sim - began > 25:
                print("nothing from the sim", flush=True)
                os._exit(3)
    sim = node.get_clock().now().nanoseconds * 1e-9 - node.start
    wall = time.time() - began_sim
    print(f"{sim:.1f} s of sim time in {wall:.1f} s, {sim / wall:.1f} times "
          f"real time", flush=True)
    # gz.transport aborts as it is torn down at exit, so skip that.
    os._exit(0)


if __name__ == "__main__":
    main()
