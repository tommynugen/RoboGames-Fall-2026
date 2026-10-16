"""Reads a PlayStation controller's motion sensor through Linux joydev.

hid-playstation gives each DualShock 4 or DualSense a second input device,
"... Motion Sensors", whose first three axes are the accelerometer and the
next three the gyro. joy_node only opens the gamepad itself, so this reads
the sensor's /dev/input/jsN directly. The js node is readable without being
in the input group, where the matching eventN is not.
"""

import glob
import os
import struct

JS_EVENT_FORMAT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_AXIS = 0x02

ACCEL_AXES = (0, 1, 2)


def _read_sysfs(path):
    try:
        with open(path) as sysfs_file:
            return sysfs_file.read().strip()
    except OSError:
        return ""


def find_motion_sensors():
    """Lists (device path, name, Bluetooth address) for each sensor."""
    sensors = []

    for sysfs in sorted(glob.glob("/sys/class/input/js*")):
        name = _read_sysfs(os.path.join(sysfs, "device", "name"))

        if not name.endswith("Motion Sensors"):
            continue

        address = _read_sysfs(os.path.join(sysfs, "device", "uniq"))
        device = os.path.join("/dev/input", os.path.basename(sysfs))
        sensors.append((device, name, address.upper()))

    return sensors


class MotionSensor:

    def __init__(self, device):
        self.device = device
        self.axes = [0, 0, 0, 0, 0, 0]
        self.fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)

    def poll(self):
        """Reads every waiting event. Returns False once the device is gone."""
        while True:
            try:
                data = os.read(self.fd, JS_EVENT_SIZE * 64)
            except BlockingIOError:
                return True
            except OSError:
                return False

            if not data:
                return False

            for start in range(0, len(data) - JS_EVENT_SIZE + 1,
                               JS_EVENT_SIZE):
                _, value, kind, number = struct.unpack(
                    JS_EVENT_FORMAT, data[start:start + JS_EVENT_SIZE])

                if kind & JS_EVENT_AXIS and number < len(self.axes):
                    self.axes[number] = value

    def gravity(self):
        return tuple(self.axes[axis] for axis in ACCEL_AXES)

    def close(self):
        os.close(self.fd)
