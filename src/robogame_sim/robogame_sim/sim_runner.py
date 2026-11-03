"""Starts and stops the Gazebo sim for the RoboGame app's Launch buttons.

The app runs one `ros2 launch robogame_sim sim.launch.py layout:=NAME ...`
at a time, as its own process group, with its output in a log file in the
temp folder. Stop sends Ctrl-C (SIGINT) to the whole group, as the
terminal would, then SIGKILL if it is still up 10 s later. Quitting the app
stops the sim too.

Routes, served by gamemaster.py:

- GET /api/sim: {"running", "layout", "command", "log", "tail"}
- POST /api/sim {"layout": NAME, "test": bool}: launches it
- POST /api/sim/stop: stops it
"""

import os
import shutil
import signal
import subprocess
import tempfile
import threading

from robogame_sim import arena_layout

# As the editor's Run it and Test it commands.
RUN = ["robots:=1,2", "chase:=1,2"]
TEST = RUN + ["controller:=hotplug", "drive:=2"]
STOP_WAIT = 10.0  # s, from Ctrl-C to SIGKILL
TAIL = 2000  # bytes of log sent back


class SimRunner:
    def __init__(self, launcher="ros2"):
        self.launcher = launcher
        self.process = None
        self.layout = None
        self.command = None
        self.log = os.path.join(tempfile.gettempdir(), "robogame_sim.log")
        self.lock = threading.Lock()

    def running(self):
        return self.process is not None and self.process.poll() is None

    def status(self):
        tail = ""
        if self.command and os.path.isfile(self.log):
            with open(self.log, "rb") as file:
                file.seek(max(0, os.path.getsize(self.log) - TAIL))
                tail = file.read().decode(errors="replace")
        return {"running": self.running(), "layout": self.layout,
                "command": " ".join(self.command or []), "log": self.log,
                "tail": tail,
                "exit": None if self.process is None
                else self.process.poll()}

    def start(self, layout, test=False):
        if not arena_layout.NAME.match(layout or ""):
            raise ValueError("names are letters, digits, - and _ only")
        arena_layout.find_layout(layout)  # FileNotFoundError if not saved
        if shutil.which(self.launcher) is None:
            raise ValueError(f"no {self.launcher} on the PATH: source ROS "
                             "and the workspace, then start the app again")
        with self.lock:
            if self.running():
                raise ValueError(f"the sim is already running "
                                 f"{self.layout}: stop it first")
            command = [self.launcher, "launch", "robogame_sim",
                       "sim.launch.py", f"layout:={layout}"] + \
                (TEST if test else RUN)
            with open(self.log, "wb") as log:
                self.process = subprocess.Popen(
                    command, stdout=log, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, start_new_session=True)
            self.layout, self.command = layout, command
        return self.status()

    def stop(self):
        with self.lock:
            process = self.process
            if process is None or process.poll() is not None:
                return self.status()
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
        return None

    def post(self, path, read_body):
        try:
            if path == "/api/sim":
                body = read_body()
                return 200, self.start(body.get("layout"),
                                       bool(body.get("test")))
            if path == "/api/sim/stop":
                return 200, self.stop()
        except (ValueError, FileNotFoundError) as error:
            return 400, {"error": str(error)}
        return None
