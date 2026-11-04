"""Starts and stops the Gazebo sim for the RoboGame app's Launch buttons.

The app runs one `ros2 launch robogame_sim sim.launch.py layout:=NAME ...`
at a time, as its own process group, with its output in a log file in the
temp folder. Stop sends Ctrl-C (SIGINT) to the whole group, as the
terminal would, then SIGKILL if it is still up 10 s later. Quitting the app
stops the sim too.

Routes, served by gamemaster.py:

- GET /api/sim: {"running", "ready", "test", "layout", "command", "log",
  "tail", "exit", "stopped"}. ready is false while it is still
  launching: until a robot is spawned in the world, or READY_AFTER
  seconds at most. stopped is true once Stop ended it
- POST /api/sim {"layout": NAME, "test": bool, "options": {...}}:
  launches it. options are sim.launch.py's arguments by name (robots,
  chase, gui, fpv, speed, step, controller, drive, drive_model, autonomy,
  joy_device_id, motion_device, partition), each checked here first so a
  bad one is a 400 with a reason rather than a launch that dies. They go
  over RUN, and TEST when test is true; an empty one is left out, so
  chase "" runs no chase. GET gives back the ones used as "options"
- POST /api/sim/stop: stops it
- GET /api/sim/view and POST /api/sim/view {"view", "robot"}: the
  Gazebo window's camera, by sim_camera.py
"""

import os
import re
import shutil
import signal
import subprocess
import tempfile
import threading
import time

from robogame_sim import arena_layout
from robogame_sim.sim_camera import SimCamera

# As the editor's Run it and Test it commands.
RUN = {"robots": "1,2", "chase": "1,2"}
TEST = dict(RUN, controller="hotplug", drive="2")
CONTROLLERS = ("none", "joy", "classic", "hotplug")
DRIVE_MODELS = ("ideal", "firmware")
FLAGS = ("gui", "fpv", "autonomy")
STOP_WAIT = 10.0  # s, from Ctrl-C to SIGKILL
TAIL = 2000  # bytes of log sent back
# What ros_gz_sim create prints once a robot is in the world, in its newer
# and older wording, which means Gazebo is up.
READY = (b"Entity creation successful", b"OK creation of entity")
READY_AFTER = 90.0  # s, called running then whatever the log says


class SimRunner:
    def __init__(self, launcher="ros2"):
        self.launcher = launcher
        self.process = None
        self.layout = None
        self.command = None
        self.test = False
        self.options = {}
        self.started = 0.0
        self.ready = False
        self.stopped = False  # by Stop, so its exit code is no failure
        self.log = os.path.join(tempfile.gettempdir(), "robogame_sim.log")
        self.lock = threading.Lock()
        self.camera = SimCamera(lambda: self.options.get("partition", ""))

    def running(self):
        return self.process is not None and self.process.poll() is None

    def is_ready(self):
        """Whether the sim is up, not still launching."""
        if not self.ready and self.running():
            try:
                with open(self.log, "rb") as file:
                    text = file.read()
            except OSError:
                text = b""
            self.ready = (any(mark in text for mark in READY)
                          or time.monotonic() - self.started > READY_AFTER)
        return self.ready

    def status(self):
        tail = ""
        if self.command and os.path.isfile(self.log):
            with open(self.log, "rb") as file:
                file.seek(max(0, os.path.getsize(self.log) - TAIL))
                tail = file.read().decode(errors="replace")
        return {"running": self.running(), "ready": self.is_ready(),
                "test": self.test, "layout": self.layout,
                "options": self.options,
                "command": " ".join(self.command or []), "log": self.log,
                "tail": tail, "stopped": self.stopped,
                "exit": None if self.process is None
                else self.process.poll()}

    def start(self, layout, test=False, options=None):
        layout = arena_layout.layout_name(layout)
        arena_layout.find_layout(layout)  # FileNotFoundError if not saved
        used = launch_options(test, options)  # ValueError for a bad one
        if shutil.which(self.launcher) is None:
            raise ValueError(f"no {self.launcher} on the PATH: source ROS "
                             "and the workspace, then start the app again")
        with self.lock:
            if self.running():
                raise ValueError(f"the sim is already running "
                                 f"{self.layout}: stop it first")
            command = [self.launcher, "launch", "robogame_sim",
                       "sim.launch.py", f"layout:={layout}"] + \
                [f"{name}:={value}" for name, value in used.items()]
            with open(self.log, "wb") as log:
                self.process = subprocess.Popen(
                    command, stdout=log, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, start_new_session=True)
            self.layout, self.command, self.test = layout, command, test
            self.options = used
            self.started, self.ready = time.monotonic(), False
            self.stopped = False
        return self.status()

    def stop(self):
        with self.lock:
            process = self.process
            if process is None or process.poll() is not None:
                return self.status()
            self.stopped = True
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(STOP_WAIT)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            except ProcessLookupError:
                pass
        return self.status()

    def get(self, path):
        if path == "/api/sim":
            return 200, self.status()
        return self.camera.get(path)

    def post(self, path, read_body):
        try:
            if path == "/api/sim":
                body = read_body()
                return 200, self.start(body.get("layout"),
                                       bool(body.get("test")),
                                       body.get("options"))
            if path == "/api/sim/stop":
                return 200, self.stop()
        except (ValueError, FileNotFoundError) as error:
            return 400, {"error": str(error)}
        return self.camera.post(path, read_body)


def robot_list(text, name):
    try:
        robots = [int(part) for part in str(text).split(",")
                  if part.strip()]
    except ValueError:
        raise ValueError(f"{name} takes robot numbers such as 1,2, "
                         f"not {text!r}") from None
    if any(not 1 <= robot <= 9 for robot in robots):
        raise ValueError(f"{name} takes robots 1 to 9")
    return robots


def number(text, name, low, high):
    try:
        value = float(text)
    except (TypeError, ValueError):
        raise ValueError(f"{name} takes a number, not {text!r}") from None
    if not low <= value <= high:
        raise ValueError(f"{name} takes {low} to {high}, not {value:g}")
    return f"{value:g}"


def launch_options(test=False, options=None):
    """sim.launch.py's arguments for a run, name to value: RUN, or TEST,
    with options over them, each checked as the launch file would, and
    the empty ones left out."""
    used = dict(TEST if test else RUN)
    if options is None:
        options = {}
    if not isinstance(options, dict):
        raise ValueError("options must be an object of launch arguments")
    for name, value in options.items():
        if value is None:
            continue
        if isinstance(value, bool):
            value = "true" if value else "false"
        value = str(value).strip()
        if name in FLAGS:
            if value not in ("true", "false"):
                raise ValueError(f"{name} takes true or false")
        elif name == "robots":
            robots = robot_list(value, name)
            if not robots:
                raise ValueError("robots needs at least one robot")
            value = ",".join(map(str, robots))
        elif name == "chase":
            chase = robot_list(value, name)
            if chase and (len(chase) != 2 or chase[0] == chase[1]):
                raise ValueError("chase takes a hunter and a target, "
                                 "such as 1,2")
            value = ",".join(map(str, chase))
        elif name == "drive":
            drive = robot_list(value, name)
            if len(drive) > 1:
                raise ValueError("drive takes one robot")
            value = ",".join(map(str, drive))
        elif name == "controller":
            if value not in CONTROLLERS:
                raise ValueError("controller takes " + ", ".join(CONTROLLERS))
        elif name == "drive_model":
            if value not in DRIVE_MODELS:
                raise ValueError("drive_model takes ideal or firmware")
        elif name == "speed":
            value = number(value, name, 0, 100)
        elif name == "step":
            value = number(value, name, 0.0001, 0.01)
        elif name == "joy_device_id":
            if not value.isdigit():
                raise ValueError("joy_device_id takes a number such as 0")
        elif name == "motion_device":
            if value and not re.fullmatch(r"/dev/input/[\w.-]+", value):
                raise ValueError("motion_device takes a path such as "
                                 "/dev/input/js1")
        elif name == "partition":
            if not re.fullmatch(r"[\w-]*", value):
                raise ValueError("partition takes letters, digits, _ and -")
        else:
            raise ValueError(f"no launch option {name}")
        used[name] = value
    used = {name: value for name, value in used.items() if value != ""}

    robots = robot_list(used["robots"], "robots")
    for name in ("chase", "drive"):
        missing = [robot for robot in robot_list(used.get(name, ""), name)
                   if robot not in robots]
        if missing:
            raise ValueError(f"{name} names robot {missing[0]}, which "
                             f"robots ({used['robots']}) does not spawn")
    return used
