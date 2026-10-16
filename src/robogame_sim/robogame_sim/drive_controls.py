"""Maths for racing-game style driving, kept apart from ROS so it can be
tested on its own.

Triggers come from joy as 1.0 released and -1.0 fully pressed. Sticks are
-1.0 to 1.0 with left and up positive, the same way a Twist counts a left
turn and forward.

The motion sensor is read as a gravity vector in the controller's own axes,
as hid-playstation reports it: X through the grips (right positive), Y out
of the face (up when the controller lies flat) and Z toward the player.
"""

import math


def trigger_pressed(value, deadzone=0.03):
    """How far a trigger is pressed, 0.0 to 1.0."""
    pressed = (1.0 - value) / 2.0

    if pressed <= deadzone:
        return 0.0

    return min(1.0, (pressed - deadzone) / (1.0 - deadzone))


def past_deadzone(value, deadzone, full):
    """Scales value past its deadzone so full gives 1.0, keeping its sign."""
    magnitude = abs(value)

    if magnitude <= deadzone:
        return 0.0

    scaled = min(1.0, (magnitude - deadzone) / (full - deadzone))
    return math.copysign(scaled, value)


def drive(throttle, reverse, turn, car_reverse_steering=True):
    """Forward and turn, each -1.0 to 1.0, from the triggers and a turn.

    R2 drives forward and L2 backward, so pressing both cancels out. Like a
    car, a left turn while reversing swings the back of the robot left,
    which turns it the other way.
    """
    forward = throttle - reverse

    if car_reverse_steering and forward < 0.0:
        turn = -turn

    return forward, max(-1.0, min(1.0, turn))


def roll_degrees(gravity):
    """Roll about the controller's long axis. Left grip down is positive."""
    x, y, z = gravity
    return math.degrees(math.atan2(x, math.hypot(y, z)))


def tilt_up_degrees(gravity, centre):
    """How far the far end has tipped up since centre, about the X axis."""
    _, y, z = gravity
    _, centre_y, centre_z = centre
    cross = centre_z * y - centre_y * z
    dot = centre_y * y + centre_z * z
    return math.degrees(math.atan2(cross, dot))


class SteeringWheel:
    """Turns the controller's roll into a turn and its tilt into arm moves.

    The way the controller is held when centred is straight ahead and level.
    """

    def __init__(self, full_degrees=45.0, deadzone_degrees=4.0,
                 arm_tilt_degrees=20.0, smoothing=0.25,
                 steer_inverted=False, arm_inverted=False):
        self.full_degrees = full_degrees
        self.deadzone_degrees = deadzone_degrees
        self.arm_tilt_degrees = arm_tilt_degrees
        self.smoothing = smoothing
        self.steer_inverted = steer_inverted
        self.arm_inverted = arm_inverted

        self.gravity = None
        self.centre = None
        self.centre_roll = 0.0

    def update(self, reading):
        """Feeds in a new accelerometer reading, smoothing it."""
        if self.gravity is None:
            self.gravity = tuple(reading)
            return

        self.gravity = tuple(
            old + (new - old) * self.smoothing
            for old, new in zip(self.gravity, reading)
        )

    def has_reading(self):
        return self.gravity is not None and any(self.gravity)

    def recentre(self):
        self.centre = self.gravity
        self.centre_roll = roll_degrees(self.gravity)

    def roll(self):
        roll = roll_degrees(self.gravity) - self.centre_roll
        return -roll if self.steer_inverted else roll

    def tilt_up(self):
        tilt = tilt_up_degrees(self.gravity, self.centre)
        return -tilt if self.arm_inverted else tilt

    def turn(self):
        """-1.0 to 1.0, positive left."""
        return past_deadzone(
            self.roll(), self.deadzone_degrees, self.full_degrees)

    def shoulder(self):
        """1 to raise the shoulder, -1 to lower it, 0 to hold it."""
        tilt = self.tilt_up()

        if tilt > self.arm_tilt_degrees:
            return 1

        if tilt < -self.arm_tilt_degrees:
            return -1

        return 0
