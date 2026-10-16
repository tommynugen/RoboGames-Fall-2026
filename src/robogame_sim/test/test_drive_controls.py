import math

from robogame_sim.drive_controls import (
    SteeringWheel,
    drive,
    past_deadzone,
    roll_degrees,
    tilt_up_degrees,
    trigger_pressed,
)

FLAT = (0.0, 8192.0, 0.0)


def rolled(degrees):
    # Left grip down tips the +X axis up, so gravity shows along +X.
    angle = math.radians(degrees)
    return (8192.0 * math.sin(angle), 8192.0 * math.cos(angle), 0.0)


def tipped_up(degrees):
    # Far end up tips +Z, toward the player, down.
    angle = math.radians(degrees)
    return (0.0, 8192.0 * math.cos(angle), -8192.0 * math.sin(angle))


def test_triggers_read_released_to_fully_pressed():
    assert trigger_pressed(1.0) == 0.0
    assert trigger_pressed(0.97) == 0.0
    assert trigger_pressed(-1.0) == 1.0
    assert abs(trigger_pressed(0.0) - 0.4845) < 0.001


def test_stick_deadzone_scales_from_zero():
    assert past_deadzone(0.05, 0.08, 1.0) == 0.0
    assert past_deadzone(-1.0, 0.08, 1.0) == -1.0
    assert abs(past_deadzone(0.54, 0.08, 1.0) - 0.5) < 1e-9


def test_r2_forward_l2_reverse():
    assert drive(1.0, 0.0, 0.0) == (1.0, 0.0)
    assert drive(0.0, 1.0, 0.0) == (-1.0, 0.0)
    assert drive(1.0, 1.0, 0.0) == (0.0, 0.0)


def test_reversing_steers_like_a_car():
    assert drive(0.5, 0.0, 0.4) == (0.5, 0.4)
    assert drive(0.0, 0.5, 0.4) == (-0.5, -0.4)
    assert drive(0.0, 0.5, 0.4, car_reverse_steering=False) == (-0.5, 0.4)


def test_stationary_steering_spins_in_place():
    assert drive(0.0, 0.0, -0.7) == (0.0, -0.7)


def test_roll_and_tilt_angles():
    assert abs(roll_degrees(FLAT)) < 1e-9
    assert abs(roll_degrees(rolled(30)) - 30) < 1e-9
    assert abs(tilt_up_degrees(tipped_up(25), FLAT) - 25) < 1e-9
    assert abs(tilt_up_degrees(tipped_up(-25), FLAT) + 25) < 1e-9


def make_wheel(start, **settings):
    wheel = SteeringWheel(smoothing=1.0, **settings)
    wheel.update(start)
    wheel.recentre()
    return wheel


def test_wheel_turns_left_when_rolled_left():
    wheel = make_wheel(FLAT)

    wheel.update(rolled(2))
    assert wheel.turn() == 0.0

    wheel.update(rolled(24.5))
    assert abs(wheel.turn() - 0.5) < 1e-9

    wheel.update(rolled(-60))
    assert wheel.turn() == -1.0


def test_wheel_centre_is_how_it_was_held():
    wheel = make_wheel(rolled(20))

    wheel.update(rolled(20))
    assert wheel.turn() == 0.0

    wheel.update(rolled(70))
    assert wheel.turn() == 1.0


def test_tipping_moves_the_shoulder():
    wheel = make_wheel(tipped_up(-30))

    wheel.update(tipped_up(-30))
    assert wheel.shoulder() == 0

    wheel.update(tipped_up(-5))
    assert wheel.shoulder() == 1

    wheel.update(tipped_up(-55))
    assert wheel.shoulder() == -1


def test_inverted_settings_flip_directions():
    wheel = make_wheel(FLAT, steer_inverted=True, arm_inverted=True)
    wheel.update(rolled(50))
    assert wheel.turn() == -1.0

    wheel.update(tipped_up(30))
    assert wheel.shoulder() == -1


def test_smoothing_eases_toward_reading():
    wheel = SteeringWheel(smoothing=0.5)
    wheel.update((0.0, 0.0, 0.0))
    assert not wheel.has_reading()

    wheel.update((100.0, 0.0, 0.0))
    assert wheel.gravity == (50.0, 0.0, 0.0)
