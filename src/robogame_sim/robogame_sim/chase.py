"""Steering for chase_node, kept free of ROS so it can be tested.

Chaser works out a twist for the hunter each tick from both robots' poses.
The hunter drives with its arm stowed, STOW_ARM, so its footprint is just
its body and it fits through any gap its body does. It always heads for the
target: straight at it, a little ahead of where it is going, when the way
is clear, and along a path planned over the terrain when it is not. It
turns on the spot only where it has room to and the turn is large, lines
up before driving into a gap too narrow to turn in, backs out of one when
the target goes the other way, and frees itself when it has been stuck.
"""

import math
from collections import deque

from robogame_sim.robot_sdf import (
    FINGER_LENGTH,
    FOREARM_LENGTH,
    SHOULDER,
    UPPER_ARM_LENGTH,
    WHEEL_X,
)
from robogame_sim.terrain import (
    DROP_HALF_WIDTH,
    HALF_LENGTH,
    HALF_WIDTH,
    SOFT_CLEARANCE,
    TURN_CLEARANCE,
    Path,
    corner_angle,
    next_waypoint,
    path_length,
    point_along,
)

# An arm at its start angles, level and straight, reaches well past the
# chassis. Stowed, with the shoulder straight up and the elbow folded back,
# it is over the chassis.
ARM_REACH = SHOULDER[0] + UPPER_ARM_LENGTH + FOREARM_LENGTH + FINGER_LENGTH

# Arm commands, one character each as the BLE bridge takes them, that stow
# the arm from any angle: shoulder and elbow each step to 180 degrees and
# stop there. STOWED is near enough in radians for both joints, as the sim
# reports them.
STOW_ARM = "+X" * 36
STOWED = math.radians(85)

# The floor tiles end at +-0.6096 m, 12 modules of 0.1016 m, with no walls
# round them.
# Predicted points are kept this far inside the edge, since a target is
# not going to drive off it and the hunter should not be sent over it.
ARENA_HALF = 6 * 0.1016
WALL_MARGIN = 0.06

KEEP_UP_FADE = 0.02
# Fastest it turns where walls leave it no room to turn on the spot, and
# where only a drop does, as it pivots with its wheels inside WHEEL_REACH.
NARROW_TURN_RATE = 0.6
PIVOT_TURN_RATE = 1.0
# How far along the path it looks lined up in a gap, up to the first
# corner sharper than LANE_CORNER.
LANE_LOOKAHEAD = 0.12
LANE_CORNER = math.radians(20)
# A corner onto a ramp sharper than this it stops on to line up.
RAMP_CORNER = math.radians(10)
# How far back from a corner it may stop to line up for a gap, to have
# more room to turn.
TURN_BACK = 0.05
# Following a path it curves at no more than this share of its top turn
# rate, slowing for a tighter curve, so the turn keeps up with the curve.
CURVE_TURN = 0.8
WHEEL_REACH = math.hypot(WHEEL_X, HALF_WIDTH)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def arm_reach(shoulder, elbow):
    """How far forward of a robot's centre its arm or body reaches, for
    shoulder and elbow joint angles in radians, 0 being level and
    straight and positive raising them."""
    elbow_x = SHOULDER[0] + UPPER_ARM_LENGTH * math.cos(shoulder)
    tip_x = elbow_x + (FOREARM_LENGTH + FINGER_LENGTH) * math.cos(
        shoulder + elbow)
    return max(HALF_LENGTH, elbow_x, tip_x)


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


def footprint_extent(bearing, reach=ARM_REACH):
    """Distance from a robot's centre to the edge of its footprint, along
    bearing radians from its heading, for an arm or body reaching reach
    metres forward."""
    dx, dy = math.cos(bearing), math.sin(bearing)
    reach_x = reach if dx > 0 else HALF_LENGTH

    return min(
        reach_x / abs(dx) if abs(dx) > 1e-9 else math.inf,
        HALF_WIDTH / abs(dy) if abs(dy) > 1e-9 else math.inf)


def standoff_distance(hunter_x, hunter_y, target_x, target_y, target_yaw,
                      clearance, target_reach=ARM_REACH):
    """Centre to centre distance that leaves the front of the hunter, its
    arm stowed, clearance metres short of the target, whichever way the
    target faces and however far its arm reaches."""
    bearing = normalize_angle(
        math.atan2(hunter_y - target_y, hunter_x - target_x) - target_yaw)

    return HALF_LENGTH + footprint_extent(bearing, target_reach) + clearance


def drive_command(hunter_x, hunter_y, hunter_yaw, waypoint_x, waypoint_y,
                  remaining, limit, max_angular, linear_gain, angular_gain,
                  turn_in_place, can_turn=True, line_up=False, keep_up=0.0):
    """Twist (linear, angular, heading_error) toward a waypoint, slowing
    as remaining, the distance left to drive, runs out, plus keep_up to
    match a moving target's speed.

    Forward speed falls off with heading error, so the hunter curves in.
    Past turn_in_place radians it turns on the spot, or past a few degrees
    when line_up says it is about to drive into a gap it cannot turn in.
    Where it cannot turn, can_turn False, it is lined up in such a gap: it
    backs out when the waypoint is behind it, and only creeps while it
    straightens up.
    """
    heading_error = normalize_angle(
        math.atan2(waypoint_y - hunter_y, waypoint_x - hunter_x)
        - hunter_yaw)
    error = heading_error
    reverse = not can_turn and abs(error) > math.pi / 2

    if reverse:
        error = normalize_angle(error - math.pi)

    angular = clamp(angular_gain * error, -max_angular, max_angular)
    speed = clamp(linear_gain * remaining + keep_up, 0.0, limit)
    threshold = 0.15 if line_up or not can_turn else turn_in_place

    if abs(error) <= threshold:
        linear = speed * math.cos(error)
    elif can_turn:
        linear = 0.0
    else:
        linear = min(speed, 0.03)

    if remaining < 0.01 and abs(keep_up) < 0.01:
        # A small dead band, so the hunter sits still at the standoff
        # instead of rocking back and forth on pose noise.
        linear = 0.0

    if not can_turn:
        # In a gap it cannot turn in it only straightens up, gently, as
        # it goes: spun hard, skidding wheels swing it into the side.
        angular = 0.0 if linear == 0.0 else clamp(
            angular, -NARROW_TURN_RATE, NARROW_TURN_RATE)

    return (-linear if reverse else linear), angular, heading_error


def lane_ahead(path):
    """The start of a path, as far as its first real corner: the little
    jogs between cell centres along a gap do not count."""
    for k in range(1, len(path) - 1):
        if (math.dist(path[0], path[k]) > 0.03
                and corner_angle(path[k - 1], path[k], path[k + 1])
                > LANE_CORNER):
            return path[:k + 1]
    return path


class Chaser:
    """Steers a hunter after a target over a Terrain, or straight at it
    with none. Feed it the target's poses with track, and call update with
    the hunter's each tick."""

    def __init__(self, terrain=None, clearance=0.03, max_linear=0.3,
                 max_angular=1.5, linear_gain=1.2, angular_gain=3.0,
                 turn_in_place=1.0, lead_time=0.6, approach_speed=0.25,
                 ramp_speed=0.15, narrow_speed=0.15, stuck_time=1.0):
        self.terrain = terrain
        self.clearance = clearance
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.linear_gain = linear_gain
        self.angular_gain = angular_gain
        self.turn_in_place = turn_in_place
        self.lead_time = lead_time
        self.approach_speed = approach_speed
        self.ramp_speed = ramp_speed
        self.narrow_speed = narrow_speed
        self.stuck_time = stuck_time

        self.target = TargetTracker()
        # How far forward the target's arm reaches. Taken to be out until
        # set_target_arm says otherwise.
        self.target_reach = ARM_REACH
        self.history = deque()
        self.recovery = None
        self.last_recovery = None
        self.waiting = False
        # Where the hunter last stopped to line up, and how far back from
        # the corner it stopped there to have room to turn.
        self.corner = None
        self.turn = None
        self.status = "waiting"
        self.path = None
        self.engaged = False
        self.distance = math.inf
        self.gap = math.inf

    def track(self, x, y, stamp, yaw=0.0):
        self.target.update(x, y, stamp, yaw)

    def set_target_arm(self, shoulder, elbow):
        self.target_reach = arm_reach(shoulder, elbow)

    def aim(self, x, y, distance, standoff):
        """Where to head for: where the target will be by the time the
        hunter gets there, looking no more than lead_time ahead."""
        t = self.target

        if abs(t.speed) < 0.02:
            return t.x, t.y

        lead = clamp((distance - standoff) / self.approach_speed,
                     0.0, self.lead_time)
        return t.predict(lead)

    def speed_limit(self, x, y, can_turn):
        if self.terrain is None:
            return self.max_linear

        if not can_turn:
            return self.narrow_speed

        # Slower near a wall or an edge, so a turn is not made too late.
        room = clamp((self.terrain.clearance_at(x, y) - TURN_CLEARANCE)
                     / (SOFT_CLEARANCE - TURN_CLEARANCE), 0.0, 1.0)
        return self.max_linear * (0.75 + 0.25 * room)

    def update(self, now, x, y, yaw):
        """Twist (linear, angular) for the hunter at x, y, yaw."""
        t = self.target
        terrain = self.terrain
        self.distance = distance = math.hypot(t.x - x, t.y - y)
        standoff = standoff_distance(x, y, t.x, t.y, t.yaw, self.clearance,
                                     self.target_reach)
        self.gap = gap = distance - standoff
        aim_x, aim_y = self.aim(x, y, distance, standoff)

        path = None
        if terrain is not None:
            path = terrain.route(x, y, aim_x, aim_y, t.x, t.y, yaw)
        self.path = path

        if path is not None and not path.partial and len(path) > 2:
            # The way to it goes round, as up a ramp to a target on the
            # platform right above, so it is as far as the way round: the
            # hunter is not with a target it is right under.
            self.gap = gap = max(gap, path_length(path) - standoff)

        if self.recovery is not None and now < self.recovery[0]:
            linear, angular = self.recovery[1], 0.0
            self.status = "freeing itself"
        elif terrain is not None and path is None:
            # A robot is off the arena, or the hunter is boxed in.
            linear, angular = 0.0, 0.0
            self.status = "stopped, no way to the target"
        else:
            self.recovery = None
            linear, angular = self.steer(x, y, yaw, path, aim_x, aim_y,
                                         standoff, gap)

        if terrain is not None and terrain.on_slope(x, y) and linear != 0.0:
            # Creeping up a ramp stalls on its steep top, so on a ramp it
            # goes at ramp_speed at least, either way.
            linear = math.copysign(max(abs(linear), self.ramp_speed), linear)

        self.engaged = gap < 0.03
        self.check_stuck(now, x, y, yaw, linear, angular)
        return linear, angular

    def steer(self, x, y, yaw, path, aim_x, aim_y, standoff, gap):
        t = self.target
        terrain = self.terrain
        can_turn = terrain is None or terrain.can_turn(x, y)
        limit = self.speed_limit(x, y, can_turn)
        pivot = False

        if not can_turn:
            # With only a drop too close to turn, as in a corner of the
            # floor, it can still pivot, slowly, its wheels staying on.
            wall, drop = terrain.room_at(x, y)
            pivot = can_turn = (wall >= TURN_CLEARANCE
                                and drop >= WHEEL_REACH)
        keep_up = 0.0

        self.waiting = False

        if path is not None and path.partial:
            # Out of reach, so wait as close as the hunter can get.
            wx, wy, remaining = next_waypoint(path, 0.0) if len(path) > 2 \
                else (*path[-1], path_length(path))
            if math.dist((x, y), path[-1]) < 0.03:
                wx, wy, remaining = t.x, t.y, 0.0
                self.waiting = True
            self.status = "target out of reach, going as close as possible"
        elif path is None or len(path) == 2:
            # Clear run: at the aim point, keeping up with the target.
            wx, wy = aim_x, aim_y
            remaining = math.hypot(aim_x - x, aim_y - y) - standoff
            keep_up = (t.vx * math.cos(yaw) + t.vy * math.sin(yaw)) * clamp(
                gap / KEEP_UP_FADE, 0.0, 1.0)
            self.status = "clear run at the target"
        elif not can_turn:
            # In a gap it cannot turn in it heads for a point a little way
            # along the path, which lies along the gap, rather than for a
            # cell centre that may be beside it when it has drifted off
            # the middle, and so eases back to the middle as it goes.
            wx, wy = point_along(lane_ahead(path), LANE_LOOKAHEAD)
            remaining = max(0.0, path_length(path) - standoff)
            self.status = "path"
        else:
            # A corner where the next leg goes into a gap too narrow to turn
            # in, or up or down a ramp, is where the hunter lines up for it,
            # so it stops right on it rather than cutting it.
            precise = len(path) > 2 and (
                not terrain.free_line(*path[1], *path[2])
                or (terrain.slope_line(*path[1], *path[2])
                    and corner_angle((x, y), path[1], path[2]) > RAMP_CORNER))
            reached = 0.03
            if precise:
                path = Path([path[0], self.turn_point(path, x, y)]
                            + path[2:])
                # Once on the corner it stays there while it turns, even as
                # turning shifts it a few millimetres, rather than heading
                # back onto the corner and off it again.
                corner = math.dist((x, y), path[1])
                if corner < 0.01:
                    self.corner = tuple(path[1])
                if self.corner is None or math.dist(
                        self.corner, path[1]) > 0.015 or corner > 0.03:
                    reached = 0.01
            wx, wy, remaining = next_waypoint(path, standoff, reached=reached)
            if precise and (wx, wy) == tuple(path[1]):
                remaining = min(remaining, math.dist((x, y), path[1]) + 0.005)
            elif not precise:
                # Elsewhere it curves round a corner without slowing, if the
                # way across it is open.
                ahead = self.cut_corner(path, x, y)
                if ahead is not None:
                    wx, wy = ahead
                    remaining = max(0.0, path_length(path) - standoff)
            self.status = "path"

        if gap < 0.0 and not (path is not None and path.partial):
            # Caught up, or the target came at the hunter: it holds its
            # ground facing the target rather than giving any back.
            wx, wy, remaining, keep_up = t.x, t.y, 0.0, 0.0
            self.status = "with the target"

        line_up = terrain is not None and (
            not terrain.free_line(x, y, wx, wy)
            or terrain.slope_line(x, y, wx, wy))
        linear, angular, heading_error = drive_command(
            x, y, yaw, wx, wy, remaining, limit, self.max_angular,
            self.linear_gain, self.angular_gain, self.turn_in_place,
            can_turn, line_up, keep_up)

        reach = math.hypot(wx - x, wy - y)
        if self.status == "path" and can_turn and linear > 0.0 and reach > 0.01:
            # No faster than it can turn onto the curve to the waypoint, so
            # it does not swing wide round a corner into a wall beside it.
            curvature = 2.0 * abs(math.sin(heading_error)) / reach
            if curvature > 1e-6:
                linear = min(linear,
                             CURVE_TURN * self.max_angular / curvature)

        if pivot:
            angular = clamp(angular, -PIVOT_TURN_RATE, PIVOT_TURN_RATE)
        elif (not can_turn and abs(linear) <= 0.03 and linear != 0.0
              and not self.safe_move(x, y, yaw, math.copysign(1, linear),
                                     0.04, walls=False)):
            # Creeping while it straightens up would take a wheel over an
            # edge: it turns where it is instead. A wall it may scrape.
            heading_error = normalize_angle(
                math.atan2(wy - y, wx - x) - yaw)
            if abs(heading_error) > math.pi / 2:
                heading_error = normalize_angle(heading_error - math.pi)
            linear = 0.0
            angular = clamp(self.angular_gain * heading_error,
                            -NARROW_TURN_RATE, NARROW_TURN_RATE)
        return linear, angular

    def turn_point(self, path, x, y):
        """Where to stop and turn for the path's second leg, which goes into
        a gap too narrow to turn in or onto a ramp: the corner, or a point
        back from it along the line of that leg with more room to turn, as
        the corner is often right at the edge of where the hunter can turn.
        Stopping a centimetre wide of it there would leave it unable to
        turn. Only as far back as the hunter is from the corner along that
        line, so it is never sent back to it, and where it went last time
        unless somewhere else has clearly more room."""
        corner = tuple(path[1])
        terrain = self.terrain
        ux, uy = path[2][0] - corner[0], path[2][1] - corner[1]
        length = math.hypot(ux, uy)
        if length < 1e-9:
            return corner

        ux, uy = ux / length, uy / length
        before = (corner[0] - x) * ux + (corner[1] - y) * uy
        best, room = 0.0, terrain.clearance_at(*corner)
        last = self.turn
        for k in range(1, int(min(TURN_BACK, before) / 0.005) + 1):
            back = 0.005 * k
            px, py = corner[0] - ux * back, corner[1] - uy * back
            here = terrain.clearance_at(px, py)
            if here < TURN_CLEARANCE:
                break
            if last is not None and abs(back - last) < 0.003:
                here += 0.002
            if (here > room + 1e-4 and terrain.clear_line(x, y, px, py)
                    and terrain.clear_line(px, py, *corner)):
                best, room = back, here

        self.turn = best
        return corner[0] - ux * best, corner[1] - uy * best

    def cut_corner(self, path, x, y, lookahead=(0.12, 0.09, 0.06)):
        """A point past the path's first corner, at most lookahead along
        it, that the hunter can drive straight to with room to spare, or
        None."""
        corner = math.dist((x, y), path[1])

        for distance in lookahead:
            if distance <= corner:
                return None

            px, py = point_along(path, distance)
            if self.terrain.open_line(x, y, px, py):
                return px, py

        return None

    def safe_move(self, x, y, yaw, way, distance=0.07, walls=True):
        """Whether driving distance straight forward, way 1, or back, way
        -1, ends somewhere the hunter is allowed to be, or with walls
        False, at least clear of any drop."""
        if self.terrain is None:
            return True

        end_x = x + way * distance * math.cos(yaw)
        end_y = y + way * distance * math.sin(yaw)
        wall, drop = self.terrain.room_at(end_x, end_y)
        return (not math.isnan(self.terrain.height_at(end_x, end_y))
                and (wall >= HALF_WIDTH or not walls)
                and drop >= DROP_HALF_WIDTH)

    def check_stuck(self, now, x, y, yaw, linear, angular):
        """Frees the hunter when it is pressed against something the plan
        did not allow for: after stuck_time of driving or turning without
        moving at all, or three times that without getting anywhere
        whatever it was doing, while not with the target nor waiting for
        it. It backs off for a moment, or goes forward if it was backing,
        and the other way the next time, but only a way that keeps it on
        ground it may be on."""
        history = self.history
        driving = abs(linear) > 0.03 or abs(angular) > 0.3
        history.append((now, x, y, yaw, driving, linear))

        while history and now - history[0][0] > 3 * self.stuck_time:
            history.popleft()

        if (self.recovery is not None or self.engaged or self.waiting
                or self.status == "stopped, no way to the target"):
            history.clear()
            return

        recent = [h for h in history if now - h[0] <= self.stuck_time]
        pushing = (now - recent[0][0] >= self.stuck_time * 0.9
                   and all(h[4] for h in recent)
                   and math.hypot(x - recent[0][1], y - recent[0][2]) < 0.01
                   and abs(normalize_angle(yaw - recent[0][3])) < 0.05)
        idle = (now - history[0][0] >= 3 * self.stuck_time * 0.95
                and math.hypot(x - history[0][1], y - history[0][2]) < 0.02)

        if not (pushing or idle):
            return

        push = sum(h[5] for h in recent)
        back = -1.0 if push >= 0.0 else 1.0

        if self.last_recovery is not None and now - self.last_recovery[0] < 5:
            back = -self.last_recovery[1]

        # Only where the move would leave it on ground it may be on, so it
        # never frees itself over the edge of the floor or a platform.
        safe = [way for way in (back, -back) if self.safe_move(x, y, yaw, way)]
        if not safe:
            history.clear()
            return
        back = safe[0]

        self.recovery = (now + 0.5, 0.12 * back)
        self.last_recovery = (now, back)
        history.clear()
