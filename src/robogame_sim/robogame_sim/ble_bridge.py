"""Starts and stops the real robots' BLE bridge for the Robot console.

The bridge is the repo's microcontroller_node, in arena_perception, run as
`ros2 launch arena_perception bridge.launch.py`. That package is a copy the
sync script overwrites, so it is not changed here: this reads the bridge's
log lines instead, to tell which robots it has found and whether each is
connected. parse() holds the wording it looks for, and test_ble_bridge.py
checks it against lines the bridge prints, so a change there shows up.

Every line the bridge prints goes on the Logs tab, under Bridge.

Routes, served by gamemaster.py:

- GET /api/bridge: {"running", "state", "robots", "error", "command",
  "exit"}. state is stopped, starting, searching (no robot connected yet),
  found (one or more connected) or lost (robots found, none connected now).
  robots maps each robot's number to found, connected or lost.
- POST /api/bridge {"expected_robots": N}: starts it. With N above 0 it
  stops scanning once N robots are connected.
- POST /api/bridge/stop: stops it, with Ctrl-C, as the terminal would.
"""

import os
import re
import shutil
import signal
import subprocess
import threading

STOP_WAIT = 5.0  # s, from Ctrl-C to SIGKILL

# What ble_fleet.py, ble_link.py and microcontroller_node.py print, as of
# ROBOGAME-2.0 on 2026-10-15. Each gives (event, robot number or None).
LINES = (
    (re.compile(r"Microcontroller bridge started"), "started"),
    (re.compile(r"BLE scan failed: (.*)"), "scan_failed"),
    (re.compile(r"Discovered robot (\d+) at"), "found"),
    (re.compile(r"Robot (\d+) is advertising again"), "found"),
    (re.compile(r"Connected to robot (\d+)\b"), "connected"),
    (re.compile(r"Robot (\d+) connection failed"), "lost"),
    (re.compile(r"Robot (\d+) disconnected"), "lost"),
)


def parse(line):
    """(event, robot or None, text) for a line of bridge output that
    matters, else None."""
    for pattern, event in LINES:
        match = pattern.search(line)
        if match:
            value = match.group(1) if match.groups() else None
            if event == "scan_failed":
                return event, None, value.strip()
            return event, (int(value) if value else None), ""
    return None


class BridgeRunner:
    def __init__(self, launcher="ros2", book=None):
        self.launcher = launcher
        self.book = book
        self.process = None
        self.command = None
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        self.started = False
        self.links = {}  # robot number: found, connected or lost
        self.error = ""

    def running(self):
        return self.process is not None and self.process.poll() is None

    def feed(self, line):
        """Takes one line of bridge output."""
        if self.book is not None:
            from robogame_sim.app_log import guess_level
            self.book.add("bridge", line, guess_level(line))
        parsed = parse(line)
        if parsed is None:
            return
        event, robot, text = parsed
        with self.lock:
            if event == "started":
                self.started = True
            elif event == "scan_failed":
                self.error = f"BLE scan failed: {text}"
            else:
                self.links[robot] = event
                if event == "connected":
                    self.error = ""

    def robots(self):
        """{robot number: link state} for the robots the bridge has
        found, while it runs."""
        with self.lock:
            return dict(self.links) if self.running() else {}

    def state(self):
        if not self.running():
            return "stopped"
        links = self.links.values()
        if "connected" in links:
            return "found"
        if "lost" in links:
            return "lost"
        return "searching" if self.started else "starting"

    def status(self):
        with self.lock:
            return {"running": self.running(), "state": self.state(),
                    "robots": {str(k): v for k, v in
                               sorted(self.links.items())},
                    "error": self.error,
                    "command": " ".join(self.command or []),
                    "exit": None if self.process is None
                    else self.process.poll()}

    def start(self, expected_robots=0):
        expected = int(expected_robots or 0)
        if expected < 0:
            raise ValueError("expected robots cannot be below 0")
        if shutil.which(self.launcher) is None:
            raise ValueError(f"no {self.launcher} on the PATH: source ROS "
                             "and the workspace, then start the app again")
        with self.lock:
            if self.running():
                raise ValueError("the bridge is already running")
            command = [self.launcher, "launch", "arena_perception",
                       "bridge.launch.py", f"expected_robots:={expected}"]
            # Python buffers a pipe's output, which would hold the lines
            # back until it filled.
            env = dict(os.environ, PYTHONUNBUFFERED="1",
                       RCUTILS_LOGGING_BUFFERED_STREAM="0")
            self.process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True, env=env)
            self.command = command
            self.reset()
        threading.Thread(target=self.read, args=(self.process,),
                         daemon=True).start()
        return self.status()

    def read(self, process):
        for raw in process.stdout:
            self.feed(raw.decode(errors="replace").rstrip("\r\n"))
        process.stdout.close()
        code = process.wait()
        if self.book is not None:
            self.book.add("bridge", f"The bridge stopped (exit {code})",
                          "info" if code in (0, -signal.SIGINT) else "warn")

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
        if path == "/api/bridge":
            return 200, self.status()
        return None

    def post(self, path, read_body):
        try:
            if path == "/api/bridge":
                return 200, self.start(read_body().get("expected_robots", 0))
            if path == "/api/bridge/stop":
                return 200, self.stop()
        except (ValueError, TypeError) as error:
            return 400, {"error": str(error)}
        return None
