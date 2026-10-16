"""Drives a robot from a PS4 or PS5 controller the way a racing game does.

    ros2 run robogame_sim racing_teleop --ros-args -p robot_id:=1

Stick mode (the default):
    R2 throttle, L2 reverse, left stick steers.

Wheel mode, switched on and off with Options:
    R2 throttle, L2 reverse, roll the controller like a steering wheel to
    steer, tip it up or down to raise or lower the shoulder. The way it is
    held when wheel mode starts is straight ahead. Share re-centres it.

In both, Triangle and Cross move the shoulder, Circle and Square the elbow,
R1 opens the gripper and L1 closes it.

Reads /robot_N/joy from joy_node, and the motion sensor straight from its
/dev/input/jsN. Publishes /robot_N/cmd_vel and /robot_N/arm_command, the
same topics as arena_perception's controller_input, and the current mode on
/robot_N/steering_mode.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from robogame_sim.drive_controls import (
    SteeringWheel,
    drive,
    past_deadzone,
    trigger_pressed,
)
from robogame_sim.motion_sensor import MotionSensor, find_motion_sensors

# The arm commands from PROTOCOL.md.
SHOULDER_UP = "+"
SHOULDER_DOWN = "-"
ELBOW_UP = "X"
ELBOW_DOWN = "H"
GRIPPER_OPEN = "o"
GRIPPER_CLOSE = "c"

STICK = "stick"
WHEEL = "wheel"


class RacingTeleop(Node):

    def __init__(self):
        super().__init__("racing_teleop")

        # Axis and button numbers are joy_node's for hid-playstation.
        self.declare_parameter("robot_id", 1)
        self.declare_parameter("steering_axis", 0)
        self.declare_parameter("throttle_axis", 5)
        self.declare_parameter("reverse_axis", 2)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("stick_deadzone", 0.08)
        self.declare_parameter("car_reverse_steering", True)

        self.declare_parameter("shoulder_up_button", 2)
        self.declare_parameter("shoulder_down_button", 0)
        self.declare_parameter("elbow_up_button", 1)
        self.declare_parameter("elbow_down_button", 3)
        self.declare_parameter("gripper_open_button", 5)
        self.declare_parameter("gripper_close_button", 4)
        self.declare_parameter("mode_button", 9)
        self.declare_parameter("recentre_button", 8)
        self.declare_parameter("arm_repeat_period", 0.15)

        self.declare_parameter("motion_device", "")
        self.declare_parameter("controller_address", "")
        self.declare_parameter("wheel_full_degrees", 45.0)
        self.declare_parameter("wheel_deadzone_degrees", 4.0)
        self.declare_parameter("wheel_arm_tilt_degrees", 20.0)
        self.declare_parameter("wheel_smoothing", 0.25)
        self.declare_parameter("wheel_steer_inverted", False)
        self.declare_parameter("wheel_arm_inverted", False)
        self.declare_parameter("wheel_debug", False)

        self.declare_parameter("rate", 50.0)
        self.declare_parameter("input_timeout", 0.5)

        def param(name):
            return self.get_parameter(name).value

        self.robot_id = param("robot_id")
        self.steering_axis = param("steering_axis")
        self.throttle_axis = param("throttle_axis")
        self.reverse_axis = param("reverse_axis")
        self.max_linear_speed = param("max_linear_speed")
        self.max_angular_speed = param("max_angular_speed")
        self.stick_deadzone = param("stick_deadzone")
        self.car_reverse_steering = param("car_reverse_steering")
        self.mode_button = param("mode_button")
        self.recentre_button = param("recentre_button")
        self.input_timeout = param("input_timeout")
        self.wheel_debug = param("wheel_debug")

        self.shoulder_buttons = (
            param("shoulder_up_button"), param("shoulder_down_button"))
        self.elbow_buttons = (
            param("elbow_up_button"), param("elbow_down_button"))
        self.pressed_arm_buttons = {
            param("gripper_open_button"): GRIPPER_OPEN,
            param("gripper_close_button"): GRIPPER_CLOSE,
        }

        self.wheel = SteeringWheel(
            full_degrees=param("wheel_full_degrees"),
            deadzone_degrees=param("wheel_deadzone_degrees"),
            arm_tilt_degrees=param("wheel_arm_tilt_degrees"),
            smoothing=param("wheel_smoothing"),
            steer_inverted=param("wheel_steer_inverted"),
            arm_inverted=param("wheel_arm_inverted"),
        )

        self.mode = STICK
        self.axes = []
        self.buttons = []
        self.triggers_touched = set()
        self.last_joy_time = None
        self.stopped_for_lost_input = True
        self.last_logged_drive = None
        self.debug_counter = 0

        self.motion = self.open_motion_sensor(
            param("motion_device"), param("controller_address"))

        namespace = f"/robot_{self.robot_id}"

        self.twist_publisher = self.create_publisher(
            Twist, f"{namespace}/cmd_vel", 10)
        self.arm_publisher = self.create_publisher(
            String, f"{namespace}/arm_command", 10)
        self.mode_publisher = self.create_publisher(
            String, f"{namespace}/steering_mode",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.create_subscription(Joy, "joy", self.joy_callback, 10)
        self.create_timer(1.0 / param("rate"), self.drive_tick)
        self.create_timer(param("arm_repeat_period"), self.arm_tick)

        self.publish_mode()
        self.get_logger().info(
            f"Racing controls for robot {self.robot_id}: R2 throttle, L2 "
            "reverse, left stick steers. Options switches to wheel steering.")

    def open_motion_sensor(self, device, address):
        if not device:
            sensors = find_motion_sensors()

            if address:
                sensors = [s for s in sensors if s[2] == address.upper()]

            if not sensors:
                self.get_logger().warning(
                    "No controller motion sensor found, wheel steering is "
                    "off. Is the controller connected over Bluetooth or USB?")
                return None

            device, name, found_address = sensors[0]
            self.get_logger().info(
                f"Motion sensor {name} ({found_address}) on {device}")

        try:
            return MotionSensor(device)
        except OSError as error:
            self.get_logger().warning(
                f"Cannot open {device} for wheel steering: {error}")
            return None

    def button(self, index):
        return 0 <= index < len(self.buttons) and self.buttons[index] == 1

    def axis(self, index):
        if index < 0 or index >= len(self.axes):
            return 0.0

        return self.axes[index]

    def trigger(self, index):
        # SDL can report a trigger as 0.0, half pressed, until it is first
        # moved, so it counts as released until it has been seen elsewhere.
        value = self.axis(index)

        if value != 0.0:
            self.triggers_touched.add(index)
        elif index not in self.triggers_touched:
            return 0.0

        return trigger_pressed(value)

    def joy_callback(self, message: Joy):
        self.last_joy_time = self.get_clock().now()
        self.stopped_for_lost_input = False

        buttons = list(message.buttons)
        was_down = self.button

        def pressed_now(index):
            return 0 <= index < len(buttons) and buttons[index] == 1 \
                and not was_down(index)

        if pressed_now(self.mode_button):
            self.toggle_mode()

        if pressed_now(self.recentre_button) and self.mode == WHEEL:
            self.wheel.recentre()
            self.get_logger().info("Wheel centred")

        for index, command in self.pressed_arm_buttons.items():
            if pressed_now(index):
                self.publish_arm(command)

        self.axes = list(message.axes)
        self.buttons = buttons

    def toggle_mode(self):
        if self.mode == WHEEL:
            self.mode = STICK
            self.get_logger().info("Stick steering")
        elif self.motion is None or not self.wheel.has_reading():
            self.get_logger().warning(
                "No motion sensor readings, staying on stick steering")
        else:
            self.mode = WHEEL
            self.wheel.recentre()
            self.get_logger().info(
                "Wheel steering: roll the controller to steer, tip it up or "
                "down to move the arm, Share to re-centre")

        self.publish_mode()

    def publish_mode(self):
        self.mode_publisher.publish(String(data=self.mode))

    def read_motion(self):
        if self.motion is None:
            return

        if not self.motion.poll():
            self.get_logger().warning("Motion sensor disconnected")
            self.motion.close()
            self.motion = None

            if self.mode == WHEEL:
                self.mode = STICK
                self.publish_mode()
            return

        self.wheel.update(self.motion.gravity())

    def input_lost(self):
        if self.last_joy_time is None:
            return True

        elapsed = self.get_clock().now() - self.last_joy_time
        return elapsed.nanoseconds > self.input_timeout * 1e9

    def drive_tick(self):
        self.read_motion()

        if self.input_lost():
            if not self.stopped_for_lost_input:
                # A controller that dies with the throttle held would
                # otherwise leave the robot driving.
                self.stopped_for_lost_input = True
                self.buttons = []
                self.twist_publisher.publish(Twist())
                self.get_logger().warning(
                    f"Lost controller input for robot {self.robot_id}, "
                    "stopping it")
            return

        if self.mode == WHEEL:
            turn = self.wheel.turn()
        else:
            turn = past_deadzone(
                self.axis(self.steering_axis), self.stick_deadzone, 1.0)

        forward, turn = drive(
            self.trigger(self.throttle_axis),
            self.trigger(self.reverse_axis),
            turn,
            self.car_reverse_steering,
        )

        twist = Twist()
        twist.linear.x = forward * self.max_linear_speed
        twist.angular.z = turn * self.max_angular_speed
        self.twist_publisher.publish(twist)

        self.log_drive(twist)

    def log_drive(self, twist):
        rounded = (round(twist.linear.x, 1), round(twist.angular.z, 1))

        if rounded != self.last_logged_drive:
            self.last_logged_drive = rounded
            self.get_logger().info(
                f"Robot {self.robot_id} {self.mode} "
                f"linear={twist.linear.x:.2f} angular={twist.angular.z:.2f}")

        if self.wheel_debug and self.mode == WHEEL:
            self.debug_counter += 1

            if self.debug_counter % 10 == 0:
                self.get_logger().info(
                    f"wheel roll={self.wheel.roll():.1f} "
                    f"tilt_up={self.wheel.tilt_up():.1f}")

    def arm_tick(self):
        if self.input_lost():
            return

        up, down = self.shoulder_buttons

        if self.button(up):
            self.publish_arm(SHOULDER_UP)
        elif self.button(down):
            self.publish_arm(SHOULDER_DOWN)
        elif self.mode == WHEEL:
            shoulder = self.wheel.shoulder()

            if shoulder > 0:
                self.publish_arm(SHOULDER_UP)
            elif shoulder < 0:
                self.publish_arm(SHOULDER_DOWN)

        up, down = self.elbow_buttons

        if self.button(up):
            self.publish_arm(ELBOW_UP)
        elif self.button(down):
            self.publish_arm(ELBOW_DOWN)

    def publish_arm(self, command):
        self.arm_publisher.publish(String(data=command))
        self.get_logger().info(f"Robot {self.robot_id} arm {command}")

    def destroy_node(self):
        if self.motion is not None:
            self.motion.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RacingTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
