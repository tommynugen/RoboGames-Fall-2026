"""Runs the repo's upkeep jobs for the Gamemaster's Upkeep tab.

Three jobs, one at a time, each its own process group with its output in
robogame_upkeep.log in the temp folder (the Logs tab's Upkeep source):

- onshape: scripts/export_onshape_arena.py, the arena CAD model from
  Onshape. Needs the API keys in ~/.config/onshape/keys.env.
- perception: scripts/sync_arena_perception.sh [PATH], arena_perception
  copied from a ROBOGAME-2.0 checkout (the script's own default with no
  path).
- build: colcon build --symlink-install in the workspace, which both
  of the others need after them.

The jobs run from the checkout this file is in, found by walking up from
it (the build's symlinks lead back there). An installed copy without one
cannot run them.

Routes, served by gamemaster.py:

- GET /api/upkeep: {"root", "job", "running", "exit", "stopped",
  "command", "tail", "keys", "cad", "perception", "jobs"}. cad and
  perception are when the CAD model and the arena_perception copy were
  last written (s since 1970, or null for none), keys whether the
  Onshape keys are there, jobs the names of the jobs and what they do
- POST /api/upkeep {"job": NAME, "path": PATH}: starts a job (path only
  for perception)
- POST /api/upkeep/stop: stops it
"""

import os
import shutil
import signal
import subprocess
import tempfile
import threading

STOP_WAIT = 5.0  # s, from Ctrl-C to SIGKILL
TAIL = 4000  # bytes of log sent back
KEYS = os.path.expanduser("~/.config/onshape/keys.env")
CAD = os.path.join("src", "robogame_sim", "models", "robogame_arena_cad",
                   "model.sdf")
PERCEPTION = os.path.join("src", "arena_perception", "package.xml")
JOBS = {
    "onshape": "Export the arena CAD from Onshape",
    "perception": "Copy arena_perception from ROBOGAME-2.0",
    "build": "Build the workspace (colcon)",
}


def find_root(start=__file__):
    """The checkout above start (a folder with scripts/ and src/), or
    None."""
    folder = os.path.dirname(os.path.realpath(start))
    while True:
        if os.path.isfile(os.path.join(folder, "scripts",
                                       "export_onshape_arena.py")):
            return folder
        parent = os.path.dirname(folder)
        if parent == folder:
            return None
        folder = parent


def written(path):
    try:
        return os.path.getmtime(path)
    except OSError:
        return None


class UpkeepRunner:
    def __init__(self, root=None, python="python3", colcon="colcon"):
        self.root = find_root() if root is None else root
        self.python, self.colcon = python, colcon
        self.process = None
        self.job = None
        self.command = None
        self.stopped = False
        self.log = os.path.join(tempfile.gettempdir(), "robogame_upkeep.log")
        self.lock = threading.Lock()

    def running(self):
        return self.process is not None and self.process.poll() is None

    def has_keys(self):
        return os.path.isfile(KEYS) or bool(
            os.environ.get("ONSHAPE_ACCESS_KEY")
            and os.environ.get("ONSHAPE_SECRET_KEY"))

    def status(self):
        tail = ""
        if self.command and os.path.isfile(self.log):
            with open(self.log, "rb") as file:
                file.seek(max(0, os.path.getsize(self.log) - TAIL))
                tail = file.read().decode(errors="replace")
        root = self.root
        return {"root": root, "job": self.job, "running": self.running(),
                "exit": None if self.process is None
                else self.process.poll(),
                "stopped": self.stopped,
                "command": " ".join(self.command or []), "tail": tail,
                "keys": self.has_keys(),
                "cad": written(os.path.join(root, CAD)) if root else None,
                "perception": written(os.path.join(root, PERCEPTION))
                if root else None,
                "jobs": JOBS}

    def command_for(self, job, path=None):
        scripts = os.path.join(self.root, "scripts")
        if job == "onshape":
            if not self.has_keys():
                raise ValueError(f"no Onshape API keys: put them in {KEYS}")
            return [self.python, "-u",
                    os.path.join(scripts, "export_onshape_arena.py")]
        if job == "perception":
            command = ["bash", os.path.join(scripts,
                                            "sync_arena_perception.sh")]
            if path:
                path = os.path.expanduser(str(path))
                if not os.path.isdir(os.path.join(path, "src",
                                                  "arena_perception")):
                    raise ValueError(f"no src/arena_perception in {path}")
                command.append(path)
            return command
        if job == "build":
            if shutil.which(self.colcon) is None:
                raise ValueError(f"no {self.colcon} on the PATH: source ROS"
                                 " and start the app again")
            return [self.colcon, "build", "--symlink-install"]
        raise ValueError(f"no upkeep job {job!r}")

    def start(self, job, path=None):
        if self.root is None:
            raise ValueError("this app is not run from a checkout of the "
                             "repo, so it has no scripts to run")
        command = self.command_for(job, path)
        with self.lock:
            if self.running():
                raise ValueError(f"{self.job} is still running: wait for it"
                                 " or stop it")
            with open(self.log, "wb") as log:
                log.write(f"$ {' '.join(command)}\n".encode())
                log.flush()
                self.process = subprocess.Popen(
                    command, cwd=self.root, stdout=log,
                    stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                    start_new_session=True)
            self.job, self.command, self.stopped = job, command, False
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
        if path == "/api/upkeep":
            return 200, self.status()
        return None

    def post(self, path, read_body):
        try:
            if path == "/api/upkeep":
                body = read_body()
                return 200, self.start(body.get("job"), body.get("path"))
            if path == "/api/upkeep/stop":
                return 200, self.stop()
        except ValueError as error:
            return 400, {"error": str(error)}
        return None
