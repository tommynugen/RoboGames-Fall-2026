"""Plain Python copy of what the robogame-esp firmware does with a command.

Nothing here imports ROS, so it can be tested on its own. Every constant
mirrors one in robogame-esp/src/config.h, and every function mirrors one in
motors.cpp or arm.cpp, so the simulated robot reacts to a command the same
way a real board would.
"""

import math

# robogame-esp/src/config.h
MOTOR_MIN_DUTY_PERCENT = 35
SHOULDER_START_ANGLE = 90
ELBOW_START_ANGLE = 90
ARM_ANGLE_STEP = 5
GRIPPER_OPEN_ANGLE = 105
GRIPPER_CLOSED_ANGLE = 40
DRIVE_COMMAND_TIMEOUT_MS = 750

# arena_perception/robot_commands.py
SHOULDER_UP = "+"
SHOULDER_DOWN = "-"
ELBOW_UP = "X"
ELBOW_DOWN = "H"
GRIPPER_OPEN = "o"
GRIPPER_CLOSE = "c"

# microcontroller_node defaults: these speeds are sent as 100 percent.
FULL_LINEAR_SPEED = 0.5
FULL_ANGULAR_SPEED = 1.5


def clamp(value, low, high):
    return max(low, min(high, value))


def clamp_percent(value):
    # robot_commands.clamp_percent, which the bridge applies before sending.
    return int(clamp(round(value), -100, 100))


def twist_to_percent(linear_x, angular_z,
                     full_linear_speed=FULL_LINEAR_SPEED,
                     full_angular_speed=FULL_ANGULAR_SPEED):
    """Forward and turn percentages, as the bridge packs them after 'V'."""
    return (
        clamp_percent(100.0 * linear_x / full_linear_speed),
        clamp_percent(100.0 * angular_z / full_angular_speed),
    )


def mix(forward_percent, turn_percent):
    """motors.cpp driveWithSpeed: left and right side speeds in percent."""
    forward = clamp(forward_percent, -100, 100)
    turn = clamp(turn_percent, -100, 100)

    left = forward - turn
    right = forward + turn
    largest = max(abs(left), abs(right))

    if largest > 100:
        # Integer division in C truncates toward zero.
        left = int(left * 100 / largest)
        right = int(right * 100 / largest)

    return left, right


def duty_percent(speed_percent):
    """motors.cpp dutyForSpeed, as a signed percentage of full power.

    Any non-zero speed is lifted above the stall point, so a small stick
    movement still drives the motor at 35 percent.
    """
    magnitude = min(abs(int(speed_percent)), 100)

    if magnitude == 0:
        return 0

    duty = MOTOR_MIN_DUTY_PERCENT \
        + (100 - MOTOR_MIN_DUTY_PERCENT) * magnitude // 100

    return duty if speed_percent > 0 else -duty


def side_speeds_to_twist(left_percent, right_percent, wheel_top_speed,
                         wheel_separation):
    """Body velocity for side duties, assuming speed is linear in duty."""
    left = wheel_top_speed * duty_percent(left_percent) / 100.0
    right = wheel_top_speed * duty_percent(right_percent) / 100.0

    return (left + right) / 2.0, (right - left) / wheel_separation


class ArmModel:
    """arm.cpp: servo angles in degrees, stepped by single commands."""

    def __init__(self):
        self.shoulder = SHOULDER_START_ANGLE
        self.elbow = ELBOW_START_ANGLE
        self.gripper = GRIPPER_OPEN_ANGLE

    def apply(self, command):
        """Apply every character in order, as command_handler.cpp does.

        Returns the characters that were not arm commands.
        """
        ignored = ""

        for character in command:
            if character == SHOULDER_UP:
                self.shoulder = clamp(self.shoulder + ARM_ANGLE_STEP, 0, 180)
            elif character == SHOULDER_DOWN:
                self.shoulder = clamp(self.shoulder - ARM_ANGLE_STEP, 0, 180)
            elif character == ELBOW_UP:
                self.elbow = clamp(self.elbow + ARM_ANGLE_STEP, 0, 180)
            elif character == ELBOW_DOWN:
                self.elbow = clamp(self.elbow - ARM_ANGLE_STEP, 0, 180)
            elif character == GRIPPER_OPEN:
                self.gripper = GRIPPER_OPEN_ANGLE
            elif character == GRIPPER_CLOSE:
                self.gripper = GRIPPER_CLOSED_ANGLE
            else:
                ignored += character

        return ignored

    def joint_positions(self):
        """Joint angles in radians for the simulated arm.

        A servo at 90 degrees holds its joint at zero: the shoulder level
        with the floor and the elbow straight. The gripper fingers are shut
        at the closed angle and swing out as the servo opens.
        """
        span = GRIPPER_OPEN_ANGLE - GRIPPER_CLOSED_ANGLE
        opening = (self.gripper - GRIPPER_CLOSED_ANGLE) / span

        return {
            "shoulder": math.radians(self.shoulder - 90),
            "elbow": math.radians(self.elbow - 90),
            "gripper": opening * math.radians(35),
        }
