"""Drives one robot after another, using both robots' true poses.

    ros2 run robogame_sim chase_node --ros-args \
        -p hunter_id:=1 -p target_id:=2 -p use_sim_time:=true

Reads /model/robot_N/odometry for the hunter and the target, the arena
camera's stand-in, and publishes /robot_<hunter>/cmd_vel. From the target's
poses it works out its speed and turn rate, predicts its path, and heads
for the earliest point on it the hunter can reach in time. The hunter stops
with its fingertips clearance metres short of the target, allowing for
both arms, keeps facing it, and follows when it moves.

It knows the arena's terrain from the arena model (see terrain.py): when
something is in the way it plans a path, so it goes round a platform
rather than into it, and up a ramp to a target on top of one. If either
pose goes stale the hunter is stopped.
"""

import math
import os

import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from robogame_sim.chase import (
    TargetTracker,
    chase_command,
    follow_command,
    standoff_distance,
    yaw_from_quaternion,
)
from robogame_sim import arena_layout
from robogame_sim.terrain import Terrain, next_waypoint


def default_arena_model():
    try:
        share = get_package_share_directory("robogame_sim")
    except PackageNotFoundError:
        return ""

    return os.path.join(share, "models", "robogame_arena_cad", "model.sdf")


class ChaseNode(Node):

    def __init__(self):
        super().__init__("chase_node")

        defaults = {
            "hunter_id": 1,
            "target_id": 2,
            # Gap left between the hunter's fingertips and the target.
            "clearance": 0.03,
            "max_linear_speed": 0.3,
            "max_angular_speed": 1.5,
            "linear_gain": 1.2,
            "angular_gain": 3.0,
            "turn_in_place": 1.0,
            # How far ahead the target's path is predicted, and the speed
            # the hunter counts on making when working out where it can
            # meet the target. A bit under max_linear_speed, since it
            # rarely drives flat out.
            "prediction_horizon": 1.5,
            "intercept_speed": 0.25,
            # Least forward speed on a ramp, so the hunter does not stall
            # on its steep top while closing in slowly.
            "ramp_speed": 0.15,
            "pose_timeout": 0.5,
            "rate": 20.0,
            # The arena layout, by name or path, made with arena_editor;
            # sim.launch.py passes the one it built the world from. Empty
            # for arena_model, an SDF model whose collision geometry is the
            # terrain, taken to sit at the world origin. Both empty for the
            # Onshape export installed with this package.
            "layout": "",
            "arena_model": "",
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        p = {name: self.get_parameter(name).value for name in defaults}
        self.p = p

        hunter = f"robot_{p['hunter_id']}"
        target = f"robot_{p['target_id']}"

        if hunter == target:
            raise ValueError("hunter_id and target_id must differ")

        self.hunter_pose = None
        self.hunter_stamp = None
        self.target = TargetTracker()
        self.engaged = False
        self.routing = None
        self.was_tracking = False
        self.terrain = self.load_terrain(
            p["layout"], p["arena_model"] or default_arena_model())

        self.cmd_publisher = self.create_publisher(
            Twist, f"/{hunter}/cmd_vel", 10)
        self.create_subscription(
            Odometry, f"/model/{hunter}/odometry", self.on_hunter, 10)
        self.create_subscription(
            Odometry, f"/model/{target}/odometry", self.on_target, 10)
        self.create_timer(1.0 / p["rate"], self.control_loop)

        self.get_logger().info(f"{hunter} chasing {target}")

    def load_terrain(self, layout, path):
        if layout:
            terrain = arena_layout.terrain(arena_layout.load(layout))
            self.get_logger().info(f"terrain from layout {layout}")
            return terrain

        if not path or not os.path.exists(path):
            self.get_logger().warn(
                f"no arena model at '{path}', chasing in a straight line")
            return None

        terrain = Terrain.from_sdf(path)
        self.get_logger().info(f"terrain from {path}")
        return terrain

    @staticmethod
    def stamp_seconds(message):
        return Time.from_msg(message.header.stamp).nanoseconds * 1e-9

    def on_hunter(self, message):
        pose = message.pose.pose
        self.hunter_pose = (
            pose.position.x, pose.position.y,
            yaw_from_quaternion(pose.orientation))
        self.hunter_stamp = self.stamp_seconds(message)

    def on_target(self, message):
        pose = message.pose.pose
        self.target.update(
            pose.position.x, pose.position.y, self.stamp_seconds(message),
            yaw_from_quaternion(pose.orientation))

    def fresh(self, stamp, now):
        return stamp is not None and now - stamp < self.p["pose_timeout"]

    def control_loop(self):
        p = self.p
        now = self.get_clock().now().nanoseconds * 1e-9
        tracking = (self.fresh(self.hunter_stamp, now)
                    and self.fresh(self.target.stamp, now))

        if not tracking:
            if self.was_tracking:
                self.get_logger().warn("lost a pose, stopping")
                self.cmd_publisher.publish(Twist())
            self.was_tracking = False
            return

        if not self.was_tracking:
            self.get_logger().info("tracking target")
        self.was_tracking = True

        x, y, yaw = self.hunter_pose
        distance = math.hypot(self.target.x - x, self.target.y - y)
        standoff = standoff_distance(
            x, y, self.target.x, self.target.y, self.target.yaw,
            p["clearance"])
        aim_x, aim_y, _ = self.target.intercept(
            x, y, yaw, standoff, p["intercept_speed"],
            p["max_angular_speed"], p["prediction_horizon"])

        path = None
        if self.terrain is not None:
            path = self.terrain.route(
                x, y, aim_x, aim_y, self.target.x, self.target.y)

        follow = (p["max_linear_speed"], p["max_angular_speed"],
                  p["linear_gain"], p["angular_gain"], p["turn_in_place"])

        if path is None and self.terrain is not None:
            # A robot is off the arena, or the hunter is boxed in.
            linear, angular = 0.0, 0.0
            routing = "stopped, no way to the target"
        elif path is not None and path.partial:
            # Out of reach, so wait as close as the hunter can get.
            if math.dist((x, y), path[-1]) < 0.03:
                linear, angular = 0.0, 0.0
            else:
                linear, angular, _ = follow_command(
                    x, y, yaw, *next_waypoint(path, 0.0), *follow)
            routing = "target out of reach, going as close as possible"
        elif path is None or len(path) == 2:
            linear, angular, _ = chase_command(
                x, y, yaw, aim_x, aim_y, distance,
                standoff, p["max_linear_speed"], p["max_angular_speed"],
                p["linear_gain"], p["angular_gain"], p["turn_in_place"],
                self.target.vx, self.target.vy)
            routing = "clear run at the target"
        else:
            linear, angular, _ = follow_command(
                x, y, yaw, *next_waypoint(path, standoff), *follow)
            routing = "path"

        if self.terrain is not None and self.terrain.on_slope(x, y):
            # Creeping up a ramp stalls on its steep top, and backing off
            # down it is no better, so on a ramp keep going or hold still.
            linear = 0.0 if linear <= 0.0 else max(linear, p["ramp_speed"])

        if routing != self.routing:
            self.routing = routing
            if routing == "path":
                corners = ", ".join(
                    f"({px:.2f}, {py:.2f})" for px, py in path[1:])
                self.get_logger().info(f"path to the target via {corners}")
            else:
                self.get_logger().info(routing)

        engaged = distance < standoff + 0.03
        if engaged != self.engaged:
            self.engaged = engaged
            self.get_logger().info(
                f"{'reached' if engaged else 'following'} target, "
                f"{distance:.2f} m away")

        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.cmd_publisher.publish(command)

    def stop_robot(self):
        self.cmd_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ChaseNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A SIGTERM from ros2 launch can land while spin is waiting.
        if rclpy.ok():
            raise
    finally:
        # Once launch has shut the context down nothing can be sent, and
        # the firmware's 750 ms command timeout stops the robot instead.
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
