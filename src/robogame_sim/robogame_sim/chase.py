"""Steering maths for chase_node, kept free of ROS so it can be tested.

The hunter aims at where the target will be a moment from now, turns
toward that point and drives until it is a set distance from the target.
"""

import math

from robogame_sim.robot_sdf import (
    CHASSIS,
    FINGER_LENGTH,
    FOREARM_LENGTH,
    SHOULDER,
    UPPER_ARM_LENGTH,
    WHEEL_WIDTH,
    WHEEL_Y,
)

# The arm reaches well past the chassis, so the robot's footprint runs from
# the back of the chassis to the fingertips, and out to the wheels sideways.
ARM_REACH = SHOULDER[0] + UPPER_ARM_LENGTH + FOREARM_LENGTH + FINGER_LENGTH
FOOTPRINT_BACK = CHASSIS[0] / 2
FOOTPRINT_SIDE = WHEEL_Y + WHEEL_WIDTH / 2

# Arena walls are at +-0.5 m. Aim points are kept this far inside them, so
# leading a target that runs along a wall does not send the hunter into it.
ARENA_HALF = 0.5
WALL_MARGIN = 0.06


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


class TargetTracker:
    """Smoothed target position and velocity from timestamped poses."""

    def __init__(self, smoothing=0.3):
        self.smoothing = smoothing
        self.x = self.y = None
        self.yaw = 0.0
        self.vx = self.vy = 0.0
        self.stamp = None

    def update(self, x, y, stamp, yaw=0.0):
        self.yaw = yaw

        if self.stamp is not None:
            dt = stamp - self.stamp

            if dt <= 0.0:
                return

            a = self.smoothing
            self.vx = (1 - a) * self.vx + a * (x - self.x) / dt
            self.vy = (1 - a) * self.vy + a * (y - self.y) / dt

        self.x, self.y, self.stamp = x, y, stamp

    def predict(self, lead_time, max_lead):
        """Where the target will be in lead_time seconds, at most max_lead
        metres from where it is now, and kept inside the walls."""
        dx, dy = self.vx * lead_time, self.vy * lead_time
        lead = math.hypot(dx, dy)

        if lead > max_lead:
            dx, dy = dx * max_lead / lead, dy * max_lead / lead

        limit = ARENA_HALF - WALL_MARGIN
        return (clamp(self.x + dx, -limit, limit),
                clamp(self.y + dy, -limit, limit))


def footprint_extent(bearing):
    """Distance from a robot's centre to the edge of its footprint, along
    bearing radians from its heading."""
    dx, dy = math.cos(bearing), math.sin(bearing)
    reach_x = ARM_REACH if dx > 0 else FOOTPRINT_BACK

    return min(
        reach_x / abs(dx) if abs(dx) > 1e-9 else math.inf,
        FOOTPRINT_SIDE / abs(dy) if abs(dy) > 1e-9 else math.inf)


def standoff_distance(hunter_x, hunter_y, target_x, target_y, target_yaw,
                      clearance):
    """Centre to centre distance that leaves the hunter's fingertips
    clearance metres short of the target, whichever way the target faces."""
    bearing = normalize_angle(
        math.atan2(hunter_y - target_y, hunter_x - target_x) - target_yaw)

    return ARM_REACH + footprint_extent(bearing) + clearance


def chase_command(hunter_x, hunter_y, hunter_yaw, aim_x, aim_y, distance,
                  standoff, max_linear, max_angular,
                  linear_gain, angular_gain, turn_in_place):
    """Twist (linear, angular) for the hunter.

    distance is to the target itself, aim_x and aim_y the predicted point.
    Forward speed falls off with heading error, so the hunter curves in on
    a moving target, and it turns on the spot past turn_in_place radians.
    Too close, it backs off at up to half speed.
    """
    heading_error = normalize_angle(
        math.atan2(aim_y - hunter_y, aim_x - hunter_x) - hunter_yaw)

    angular = clamp(angular_gain * heading_error, -max_angular, max_angular)

    gap = distance - standoff

    # A small dead band, so the hunter sits still at the standoff instead
    # of rocking back and forth on pose noise.
    if abs(gap) < 0.01 or abs(heading_error) > turn_in_place:
        linear = 0.0
    else:
        linear = linear_gain * gap * math.cos(heading_error)

    linear = clamp(linear, -max_linear / 2, max_linear)

    return linear, angular, heading_error
