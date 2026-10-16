"""Steering maths for chase_node, kept free of ROS so it can be tested.

The hunter predicts where the target is going from its speed and turn rate,
finds the earliest point on that path it can get to in time, and heads for
it, matching the target's speed once it has caught up.
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

# Arena walls are at +-0.5 m. Predicted points are kept this far inside
# them, since the target cannot drive through a wall and the hunter should
# not be sent into one.
ARENA_HALF = 0.5
WALL_MARGIN = 0.06

KEEP_UP_FADE = 0.02


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


class TargetTracker:
    """Smoothed target position, speed and turn rate from timestamped poses.

    The target is taken to keep its current speed and turn rate, so a robot
    driving an arc is predicted along that arc rather than off its tangent.
    """

    def __init__(self, smoothing=0.3):
        self.smoothing = smoothing
        self.x = self.y = None
        self.yaw = 0.0
        self.speed = 0.0
        self.turn_rate = 0.0
        self.stamp = None

    @property
    def vx(self):
        return self.speed * math.cos(self.yaw)

    @property
    def vy(self):
        return self.speed * math.sin(self.yaw)

    def update(self, x, y, stamp, yaw=0.0):
        if self.stamp is not None:
            dt = stamp - self.stamp

            if dt <= 0.0:
                return

            # Speed is measured along the heading half way through the
            # step, so a target driving backwards has a negative speed.
            turned = normalize_angle(yaw - self.yaw)
            mid_yaw = self.yaw + turned / 2
            speed = ((x - self.x) * math.cos(mid_yaw)
                     + (y - self.y) * math.sin(mid_yaw)) / dt

            a = self.smoothing
            self.speed = (1 - a) * self.speed + a * speed
            self.turn_rate = (1 - a) * self.turn_rate + a * turned / dt

        self.x, self.y, self.yaw, self.stamp = x, y, yaw, stamp

    def predict(self, seconds):
        """Where the target will be in seconds, kept inside the walls."""
        v, w, yaw = self.speed, self.turn_rate, self.yaw

        if abs(w) < 1e-3:
            x = self.x + v * seconds * math.cos(yaw)
            y = self.y + v * seconds * math.sin(yaw)
        else:
            end_yaw = yaw + w * seconds
            x = self.x + v / w * (math.sin(end_yaw) - math.sin(yaw))
            y = self.y + v / w * (math.cos(yaw) - math.cos(end_yaw))

        limit = ARENA_HALF - WALL_MARGIN
        return clamp(x, -limit, limit), clamp(y, -limit, limit)

    def intercept(self, hunter_x, hunter_y, hunter_yaw, standoff,
                  hunter_speed, hunter_turn_rate, horizon, step=0.05):
        """The earliest point on the target's predicted path that the
        hunter can reach in time, and how many seconds ahead it is.

        Reaching a point means getting within standoff of it, at
        hunter_speed once turned toward it at hunter_turn_rate. If no point
        within horizon seconds can be reached, the one at horizon is used.
        """
        seconds = 0.0

        if abs(self.speed) < 1e-3:
            x, y = self.predict(0.0)
            return x, y, 0.0

        while True:
            x, y = self.predict(seconds)
            distance = math.hypot(x - hunter_x, y - hunter_y)
            turn = abs(normalize_angle(
                math.atan2(y - hunter_y, x - hunter_x) - hunter_yaw))
            needed = (max(0.0, distance - standoff) / hunter_speed
                      + turn / hunter_turn_rate)

            if needed <= seconds or seconds >= horizon:
                return x, y, seconds

            seconds = min(horizon, seconds + step)


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


def chase_command(hunter_x, hunter_y, hunter_yaw, aim_x, aim_y,
                  target_distance, standoff, max_linear, max_angular,
                  linear_gain, angular_gain, turn_in_place,
                  target_vx=0.0, target_vy=0.0):
    """Twist (linear, angular) for the hunter.

    aim_x and aim_y are where the hunter means to meet the target, and
    target_distance is to where the target is now. The hunter drives to
    stop standoff short of the aim point, plus the target's own velocity
    along the hunter's heading so it keeps up rather than trailing. Forward
    speed falls off with heading error, so the hunter curves in, and it
    turns on the spot past turn_in_place radians. Closer than standoff to
    the target itself, it backs off at up to half speed.
    """
    heading_error = normalize_angle(
        math.atan2(aim_y - hunter_y, aim_x - hunter_x) - hunter_yaw)

    angular = clamp(angular_gain * heading_error, -max_angular, max_angular)

    aim_gap = math.hypot(aim_x - hunter_x, aim_y - hunter_y) - standoff
    target_gap = target_distance - standoff
    keep_up = (target_vx * math.cos(hunter_yaw)
               + target_vy * math.sin(hunter_yaw))

    # Matching the target's speed fades out over the last KEEP_UP_FADE
    # metres, so if the target stops the hunter is not carried into it.
    keep_up *= clamp(target_gap / KEEP_UP_FADE, 0.0, 1.0)

    if target_gap < 0.0:
        linear = linear_gain * target_gap
    elif abs(heading_error) > turn_in_place:
        linear = 0.0
    elif abs(aim_gap) < 0.01 and abs(keep_up) < 0.01:
        # A small dead band, so the hunter sits still at the standoff
        # instead of rocking back and forth on pose noise.
        linear = 0.0
    else:
        linear = (linear_gain * aim_gap + keep_up) * math.cos(heading_error)

    linear = clamp(linear, -max_linear / 2, max_linear)

    return linear, angular, heading_error
