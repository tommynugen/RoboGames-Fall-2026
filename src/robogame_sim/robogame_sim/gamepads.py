"""Reads gamepads through Linux joydev, one /dev/input/jsN each.

joy_node picks a controller by SDL index or by name, and every DualSense
has the same name, so it cannot hold on to one controller of several. This
finds each by its Bluetooth address instead, as the repo's gamepad_node
does, and turns its events into the axes and buttons joy_node would
publish, so racing_teleop reads it unchanged.
"""

import fcntl
import glob
import os
import re
import struct

from robogame_sim.motion_sensor import _read_sysfs

JS_EVENT_FORMAT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

# _IOR('j', 0x11, __u8) and _IOR('j', 0x12, __u8).
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12

AXIS_LIMIT = 32767.0

# A PlayStation controller has 8 axes and 13 buttons. Fewer is something
# else with a js node, like a virtual pointer or the motion sensor.
MIN_AXES = 6
MIN_BUTTONS = 10

MAC_ADDRESS = re.compile(r"^([0-9A-F]{2}:){5}[0-9A-F]{2}$")


def is_virtual(path, address):
    """Whether the js device at sysfs path, links followed, is made by
    software rather than a controller, like the X-Box pad Sunshine makes
    for a Moonlight stream or the one Steam Input makes mirroring a
    controller. Bluetooth LE controllers come through uhid too, but they
    carry their own address."""
    return (path.startswith("/sys/devices/virtual/")
            and not MAC_ADDRESS.match(address))


def convert_axis(raw_value, deadzone):
    """joy_node's scaling: the deadzone is cut out, the rest stretched back
    to 1.0, and the sign flipped so that up and left are positive."""
    value = max(float(raw_value), -AXIS_LIMIT)
    unscaled_deadzone = AXIS_LIMIT * deadzone

    if value > unscaled_deadzone:
        value -= unscaled_deadzone
    elif value < -unscaled_deadzone:
        value += unscaled_deadzone
    else:
        return 0.0

    return -value / (1.0 - deadzone) / AXIS_LIMIT


def counts(fd):
    """Number of axes and buttons of an open js device."""
    found = []

    for request in (JSIOCGAXES, JSIOCGBUTTONS):
        buffer = bytearray(1)
        fcntl.ioctl(fd, request, buffer)
        found.append(buffer[0])

    return tuple(found)


def find_gamepads():
    """Lists (device path, name, address, virtual) for each connected
    gamepad. The address is the Bluetooth one, upper case, or the device
    path for a controller that has none. virtual is is_virtual's."""
    pads = []

    for sysfs in sorted(glob.glob("/sys/class/input/js*")):
        name = _read_sysfs(os.path.join(sysfs, "device", "name"))
        device = os.path.join("/dev/input", os.path.basename(sysfs))

        if name.endswith("Motion Sensors"):
            continue

        try:
            fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            continue

        try:
            axes, buttons = counts(fd)
        except OSError:
            continue
        finally:
            os.close(fd)

        if axes < MIN_AXES or buttons < MIN_BUTTONS:
            continue

        address = _read_sysfs(os.path.join(sysfs, "device", "uniq")).upper()
        pads.append((device, name, address or device,
                     is_virtual(os.path.realpath(sysfs), address)))

    return pads


class Gamepad:

    def __init__(self, device, deadzone=0.05):
        self.device = device
        self.deadzone = deadzone
        self.axes = []
        self.buttons = []
        self.fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)

    def poll(self):
        """Reads every waiting event. Returns whether any came, and None
        once the device is gone."""
        changed = False

        while True:
            try:
                data = os.read(self.fd, JS_EVENT_SIZE * 64)
            except BlockingIOError:
                return changed
            except OSError:
                return None

            if not data:
                return None

            for start in range(0, len(data) - JS_EVENT_SIZE + 1,
                               JS_EVENT_SIZE):
                _, value, kind, number = struct.unpack(
                    JS_EVENT_FORMAT, data[start:start + JS_EVENT_SIZE])
                self.apply(kind & ~JS_EVENT_INIT, number, value)
                changed = True

    def apply(self, kind, number, value):
        if kind == JS_EVENT_AXIS:
            if number >= len(self.axes):
                self.axes.extend([0.0] * (number + 1 - len(self.axes)))
            self.axes[number] = convert_axis(value, self.deadzone)
        elif kind == JS_EVENT_BUTTON:
            if number >= len(self.buttons):
                self.buttons.extend([0] * (number + 1 - len(self.buttons)))
            self.buttons[number] = 1 if value else 0

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass
