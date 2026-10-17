"""Drives one robot after another, using both robots' true poses.

    ros2 run robogame_sim chase_node --ros-args \
        -p hunter_id:=1 -p target_id:=2 -p use_sim_time:=true

Reads /model/robot_N/odometry for the hunter and the target, the arena
camera's stand-in, and publishes /robot_<hunter>/cmd_vel. The steering is
Chaser's, in chase.py: the hunter heads for the target, a little ahead of
it when it is moving, and stops with its front clearance metres short of
it, allowing for the target's arm, then keeps facing it and follows when it
moves.

Before it sets off the hunter stows its arm, shoulder up and elbow folded
back over the chassis, on /robot_<hunter>/arm_command one step at a time,
as the BLE bridge takes them, and keeps it there. Stowed, the arm cannot
catch on a wall while the hunter turns, and the hunter fits through any
gap its body does. It holds still until the arm is up: once
/robot_<hunter>/joint_states shows it is, in the sim, or after every step
of STOW_ARM has gone out. It stows it again whenever it starts tracking
after losing a pose, in case the robot was restarted.

When /robot_<target>/joint_states comes in, as it does in the sim, the
standoff allows for where the target's arm really is; otherwise the arm is
taken to be out in front.

It knows the arena's terrain from the arena model (see terrain.py): when
something is in the way it plans a path, so it goes round a platform
rather than into it, through gaps between walls lined up with them, and up
a ramp to a target on top of one. If either pose goes stale the hunter is
stopped.
"""

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
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robogame_sim.chase import (
    STOW_ARM,
    STOWED,
    Chaser,
    yaw_from_quaternion,
)
from robogame_sim import arena_layout
from robogame_sim.terrain import Terrain


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
            # Gap left between the front of the hunter and the target.
            "clearance": 0.03,
            "max_linear_speed": 0.3,
            "max_angular_speed": 1.5,
            "linear_gain": 1.2,
            "angular_gain": 3.0,
            "turn_in_place": 1.0,
            # How far ahead of a moving target the hunter aims, at most,
            # and the speed it counts on making to get there.
            "lead_time": 0.6,
            "approach_speed": 0.25,
            # Least speed on a ramp, so the hunter does not stall on its
            # steep top, and top speed lined up in a gap too narrow to turn
            # in.
            "ramp_speed": 0.15,
            "narrow_speed": 0.15,
            # Stow the arm out of the way before setting off.
            "stow_arm": True,
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
        self.status = None
        self.engaged = False
        self.was_tracking = False
        self.arm_queue = ""
        self.hunter_arm = None
        self.chaser = Chaser(
            self.load_terrain(
                p["layout"], p["arena_model"] or default_arena_model()),
            clearance=p["clearance"],
            max_linear=p["max_linear_speed"],
            max_angular=p["max_angular_speed"],
            linear_gain=p["linear_gain"],
            angular_gain=p["angular_gain"],
            turn_in_place=p["turn_in_place"],
            lead_time=p["lead_time"],
            approach_speed=p["approach_speed"],
            ramp_speed=p["ramp_speed"],
            narrow_speed=p["narrow_speed"])

        self.cmd_publisher = self.create_publisher(
            Twist, f"/{hunter}/cmd_vel", 10)
        self.arm_publisher = self.create_publisher(
            String, f"/{hunter}/arm_command", 10)
        self.create_subscription(
            Odometry, f"/model/{hunter}/odometry", self.on_hunter, 10)
        self.create_subscription(
            Odometry, f"/model/{target}/odometry", self.on_target, 10)
        self.create_subscription(
            JointState, f"/{hunter}/joint_states", self.on_hunter_joints, 10)
        self.create_subscription(
            JointState, f"/{target}/joint_states", self.on_target_joints, 10)
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
        self.chaser.track(
            pose.position.x, pose.position.y, self.stamp_seconds(message),
            yaw_from_quaternion(pose.orientation))

    @staticmethod
    def arm_joints(message):
        """Shoulder and elbow angles from a JointState, or None."""
        joints = dict(zip(message.name, message.position))

        if "shoulder_joint" in joints and "elbow_joint" in joints:
            return joints["shoulder_joint"], joints["elbow_joint"]
        return None

    def on_hunter_joints(self, message):
        self.hunter_arm = self.arm_joints(message)

    def on_target_joints(self, message):
        arm = self.arm_joints(message)

        if arm is not None:
            self.chaser.set_target_arm(*arm)

    def fresh(self, stamp, now):
        return stamp is not None and now - stamp < self.p["pose_timeout"]

    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        tracking = (self.fresh(self.hunter_stamp, now)
                    and self.fresh(self.chaser.target.stamp, now))

        if not tracking:
            if self.was_tracking:
                self.get_logger().warn("lost a pose, stopping")
                self.cmd_publisher.publish(Twist())
            self.was_tracking = False
            return

        if not self.was_tracking:
            self.get_logger().info("tracking target")
            if self.p["stow_arm"]:
                self.arm_queue = STOW_ARM
                self.get_logger().info("stowing arm")
        self.was_tracking = True

        if self.arm_queue:
            # One step a tick: the bridge takes single commands, a few at
            # a time. Until the arm is up the hunter holds still.
            self.arm_publisher.publish(String(data=self.arm_queue[0]))
            self.arm_queue = self.arm_queue[1:]
            arm = self.hunter_arm
            if arm is not None and min(arm) >= STOWED:
                self.arm_queue = ""
            if not self.arm_queue:
                self.get_logger().info("arm stowed")
            self.cmd_publisher.publish(Twist())
            return

        chaser = self.chaser
        linear, angular = chaser.update(now, *self.hunter_pose)

        if chaser.status != self.status:
            self.status = chaser.status
            path = chaser.path
            if chaser.status == "path" and path is not None:
                corners = ", ".join(
                    f"({px:.2f}, {py:.2f})" for px, py in path[1:])
                self.get_logger().info(f"path to the target via {corners}")
            else:
                self.get_logger().info(chaser.status)

        if chaser.engaged != self.engaged:
            self.engaged = chaser.engaged
            self.get_logger().info(
                f"{'reached' if chaser.engaged else 'following'} target, "
                f"{chaser.distance:.2f} m away")

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
