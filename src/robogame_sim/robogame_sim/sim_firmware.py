"""Stands in for one robot's BLE bridge link and ESP32 firmware.

Subscribes to the same topics microcontroller_node does, so controller_input
and autonomy_node drive a simulated robot without any change:

    robot_N/cmd_vel      geometry_msgs/Twist
    robot_N/arm_command  std_msgs/String

and drives the Gazebo plugins in the robot model:

    robot_N/sim/cmd_vel  geometry_msgs/Twist  DiffDrive
    robot_N/shoulder_cmd std_msgs/Float64     JointPositionController
    robot_N/elbow_cmd    std_msgs/Float64     JointPositionController
    robot_N/gripper_cmd  std_msgs/Float64     JointPositionController

drive_model picks how cmd_vel becomes wheel motion:

    ideal     the Twist is followed as given, clamped to the bridge's full
              speeds. Good for working on autonomy.
    firmware  the Twist goes through the bridge's percentage packing and
              the firmware's mixing and 35 percent stall floor, then wheel
              speed is taken as linear in duty. Good for seeing how a real
              robot will react.

Both models stop the robot when no drive command arrives for 750 ms, as the
firmware does.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, String

from robogame_sim import firmware_model


class SimFirmwareNode(Node):

    def __init__(self):
        super().__init__("sim_firmware")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("drive_model", "ideal")
        self.declare_parameter(
            "full_linear_speed", firmware_model.FULL_LINEAR_SPEED)
        self.declare_parameter(
            "full_angular_speed", firmware_model.FULL_ANGULAR_SPEED)
        # Ground speed of one side at 100 percent duty. Measure a real robot
        # and set this to match it.
        self.declare_parameter("wheel_top_speed", 0.5)
        self.declare_parameter("wheel_separation", 0.06)
        self.declare_parameter(
            "command_timeout",
            firmware_model.DRIVE_COMMAND_TIMEOUT_MS / 1000.0)

        self.robot_id = self.get_parameter("robot_id").value
        self.drive_model = self.get_parameter("drive_model").value
        self.full_linear_speed = self.get_parameter("full_linear_speed").value
        self.full_angular_speed = \
            self.get_parameter("full_angular_speed").value
        self.wheel_top_speed = self.get_parameter("wheel_top_speed").value
        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.command_timeout = self.get_parameter("command_timeout").value

        if self.drive_model not in ("ideal", "firmware"):
            raise ValueError(
                f"drive_model must be ideal or firmware, not "
                f"{self.drive_model}")

        self.arm = firmware_model.ArmModel()
        self.last_drive_time = None
        self.timed_out = True

        self.drive_publisher = self.create_publisher(Twist, "sim/cmd_vel", 10)
        self.joint_publishers = {
            joint: self.create_publisher(Float64, f"{joint}_cmd", 10)
            for joint in ("shoulder", "elbow", "gripper")
        }

        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(
            String, "arm_command", self.on_arm_command, 10)

        self.create_timer(0.05, self.check_drive_timeout)
        # Gazebo may start after this node, so keep repeating the arm pose
        # until it has certainly been heard.
        self.create_timer(0.5, self.publish_arm)

        self.get_logger().info(
            f"Simulated robot {self.robot_id} ready, drive model "
            f"{self.drive_model}")

    def on_cmd_vel(self, message):
        self.last_drive_time = self.get_clock().now()
        self.timed_out = False

        linear, angular = self.wheel_motion(
            message.linear.x, message.angular.z)

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.drive_publisher.publish(twist)

    def wheel_motion(self, linear_x, angular_z):
        forward, turn = firmware_model.twist_to_percent(
            linear_x, angular_z,
            self.full_linear_speed, self.full_angular_speed)

        if self.drive_model == "ideal":
            return (
                self.full_linear_speed * forward / 100.0,
                self.full_angular_speed * turn / 100.0,
            )

        left, right = firmware_model.mix(forward, turn)

        return firmware_model.side_speeds_to_twist(
            left, right, self.wheel_top_speed, self.wheel_separation)

    def check_drive_timeout(self):
        if self.timed_out or self.last_drive_time is None:
            return

        elapsed = self.get_clock().now() - self.last_drive_time

        if elapsed.nanoseconds < self.command_timeout * 1e9:
            return

        self.timed_out = True
        self.drive_publisher.publish(Twist())
        self.get_logger().info(
            f"Robot {self.robot_id} heard no drive command for "
            f"{self.command_timeout:.2f} s, stopping")

    def on_arm_command(self, message):
        ignored = self.arm.apply(message.data)

        if ignored:
            self.get_logger().warning(
                f"Robot {self.robot_id} ignored unknown arm command "
                f"{ignored!r}")

        self.publish_arm()

    def publish_arm(self):
        for joint, position in self.arm.joint_positions().items():
            self.joint_publishers[joint].publish(Float64(data=position))


def main(args=None):
    rclpy.init(args=args)
    node = SimFirmwareNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A SIGTERM from ros2 launch can land while spin is waiting.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
