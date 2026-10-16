"""Drives one robot after another, using both robots' true poses.

    ros2 run robogame_sim chase_node --ros-args \
        -p hunter_id:=1 -p target_id:=2 -p use_sim_time:=true

Reads /model/robot_N/odometry for the hunter and the target, the arena
camera's stand-in, and publishes /robot_<hunter>/cmd_vel. The hunter stops
with its fingertips clearance metres short of the target, allowing for
both arms, keeps facing it, and follows when it moves. If either pose goes
stale the hunter is stopped.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from robogame_sim.chase import (
    TargetTracker,
    chase_command,
    standoff_distance,
    yaw_from_quaternion,
)


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
            "lead_time": 0.4,
            "max_lead": 0.15,
            "pose_timeout": 0.5,
            "rate": 20.0,
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
        self.was_tracking = False

        self.cmd_publisher = self.create_publisher(
            Twist, f"/{hunter}/cmd_vel", 10)
        self.create_subscription(
            Odometry, f"/model/{hunter}/odometry", self.on_hunter, 10)
        self.create_subscription(
            Odometry, f"/model/{target}/odometry", self.on_target, 10)
        self.create_timer(1.0 / p["rate"], self.control_loop)

        self.get_logger().info(f"{hunter} chasing {target}")

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
        aim_x, aim_y = self.target.predict(p["lead_time"], p["max_lead"])
        standoff = standoff_distance(
            x, y, self.target.x, self.target.y, self.target.yaw,
            p["clearance"])

        linear, angular, _ = chase_command(
            x, y, yaw, aim_x, aim_y, distance,
            standoff, p["max_linear_speed"], p["max_angular_speed"],
            p["linear_gain"], p["angular_gain"], p["turn_in_place"])

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
