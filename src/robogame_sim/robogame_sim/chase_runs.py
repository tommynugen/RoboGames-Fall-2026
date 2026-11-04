"""Runs the chase tests for the Gamemaster's Chase tab and scores them.

Two kinds of run, each one at a time, both may go at once:

- 2D: `python3 -m robogame_sim.chase_sim LAYOUT ... --json`, the hunter
  against moving targets with no Gazebo, in seconds. Each run's line is
  read back from its log as it comes.
- Gazebo: `scripts/gazebo_chase_suite.sh OUT_DIR SCENARIO ...`, the test4
  scenarios in headless Gazebo, a few minutes. Each scenario is scored
  with scripts/gazebo_chase_score.py once its run is over. The scripts
  are found from this file's real path, so they need a
  `colcon build --symlink-install` build, or ROBOGAME_SCRIPTS naming the
  folder.

Stop ends the run as Ctrl-C would, and for Gazebo the sims it started too
(their launch ids are in OUT_DIR/.launches). Quitting the app stops both.

Routes, served by gamemaster.py:

- GET /api/chase: {"2d": {...}, "gazebo": {...}, "behaviours", "scenarios"}.
  Each run has running, stopped, exit, started (time.time()), asked (what
  was asked), log, tail and its results
- POST /api/chase/2d {"layouts": [...], "seeds", "seconds", "behaviours"}
- POST /api/chase/gazebo {"scenarios": [...], "seconds", "parallel"}
- POST /api/chase/stop {"kind": "2d" | "gazebo"}
"""

import importlib.util
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

from robogame_sim import arena_layout
from robogame_sim.chase_sim import BEHAVIOURS

HERE = os.path.dirname(os.path.realpath(__file__))
SUITE = "gazebo_chase_suite.sh"
STOP_WAIT = 10.0  # s, from Ctrl-C to SIGKILL
TAIL = 3000  # bytes of log sent back
DONE = "s of sim time in"  # gazebo_chase_test.py's last line


def find_scripts():
    """The repo's scripts folder, or None when this is not a symlink
    install of a checkout."""
    named = os.environ.get("ROBOGAME_SCRIPTS")
    if named:
        return named if os.path.isfile(os.path.join(named, SUITE)) else None
    folder = HERE
    for _ in range(5):
        folder = os.path.dirname(folder)
        if os.path.isfile(os.path.join(folder, "scripts", SUITE)):
            return os.path.join(folder, "scripts")
    return None


def suite_scenarios(scripts):
    """The scenario names, from the suite's ALL= line."""
    try:
        with open(os.path.join(scripts, SUITE)) as file:
            found = re.search(r'ALL="([^"]*)"', file.read())
    except (OSError, TypeError):
        return []
    return found.group(1).split() if found else []


def number(body, key, low, high, default, kind=float):
    value = body.get(key, default)
    try:
        value = kind(value)
    except (TypeError, ValueError):
        raise ValueError(f"{key} must be a number")
    if not low <= value <= high:
        raise ValueError(f"{key} must be {low} to {high}")
    return value


class Job:
    """One subprocess, its own process group, output to a log file."""

    def __init__(self, log):
        self.log = log
        self.process = None
        self.asked = None
        self.started = None
        self.stopped = False

    def running(self):
        return self.process is not None and self.process.poll() is None

    def start(self, command, asked, env=None):
        with open(self.log, "wb") as log:
            self.process = subprocess.Popen(
                command, stdout=log, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True, env=env)
        self.asked, self.started, self.stopped = asked, time.time(), False

    def stop(self, sig=signal.SIGINT):
        process = self.process
        if process is None or process.poll() is not None:
            return
        self.stopped = True
        try:
            os.killpg(process.pid, sig)
            process.wait(STOP_WAIT)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        except ProcessLookupError:
            pass

    def text(self):
        try:
            with open(self.log, "rb") as file:
                return file.read().decode(errors="replace")
        except OSError:
            return ""

    def status(self):
        return {"running": self.running(), "stopped": self.stopped,
                "exit": None if self.process is None
                else self.process.poll(),
                "started": self.started, "asked": self.asked,
                "log": self.log, "tail": self.text()[-TAIL:]}


def summary(runs):
    """2D runs by behaviour: how many reached the target, how soon on
    average, the share of time with it after, the longest stuck."""
    rows = {}
    for run in runs:
        rows.setdefault(run["behaviour"], []).append(run)
    out = []
    for behaviour, group in rows.items():
        reached = [r["first_reach"] for r in group
                   if r["first_reach"] is not None]
        out.append({
            "behaviour": behaviour, "runs": len(group),
            "reached": len(reached),
            "mean_reach": sum(reached) / len(reached) if reached else None,
            "with_target": sum(r["with_target"] for r in group) / len(group),
            "worst_still": max(r["worst_still"] for r in group),
            "blocked": sum(r["blocked"] for r in group)})
    return out


class ChaseRunner:
    def __init__(self, scripts=None, python=sys.executable,
                 folder=None):
        self.scripts = scripts if scripts is not None else find_scripts()
        self.python = python
        folder = folder or tempfile.gettempdir()
        self.flat = Job(os.path.join(folder, "robogame_chase_2d.log"))
        self.gazebo = Job(os.path.join(folder, "robogame_chase_suite.log"))
        self.out = os.path.join(folder, "robogame_chase_suite")
        self.scenarios = suite_scenarios(self.scripts)
        self.scored = {}  # csv path: ((mtime, size), measure)
        self.lock = threading.Lock()
        self.scorer = None

    # -- 2D --------------------------------------------------------------

    def start_2d(self, body):
        layouts = body.get("layouts") or ["default"]
        if not isinstance(layouts, list):
            raise ValueError("layouts must be a list")
        layouts = [arena_layout.layout_name(name) for name in layouts]
        for name in layouts:
            arena_layout.find_layout(name)  # FileNotFoundError
        behaviours = body.get("behaviours") or list(BEHAVIOURS)
        if (not isinstance(behaviours, list)
                or not set(behaviours) <= set(BEHAVIOURS)):
            raise ValueError(f"behaviours are {', '.join(BEHAVIOURS)}")
        seeds = number(body, "seeds", 1, 20, 4, int)
        seconds = number(body, "seconds", 5, 120, 40.0)
        with self.lock:
            if self.flat.running():
                raise ValueError("a 2D chase is already running: stop it "
                                 "first")
            # The package's parent, so -m finds it however the app started.
            env = dict(os.environ)
            env["PYTHONPATH"] = os.pathsep.join(
                [os.path.dirname(HERE)]
                + [p for p in [env.get("PYTHONPATH")] if p])
            command = [self.python, "-m", "robogame_sim.chase_sim",
                       *layouts, "--seeds", str(seeds), "--seconds",
                       str(seconds), "--behaviours", ",".join(behaviours),
                       "--json"]
            self.flat.start(command, {
                "layouts": layouts, "behaviours": behaviours,
                "seeds": seeds, "seconds": seconds,
                "total": len(layouts) * len(behaviours) * seeds}, env)
        return self.status()

    def runs_2d(self):
        runs = []
        for line in self.flat.text().splitlines():
            if line.startswith("{"):
                try:
                    runs.append(json.loads(line))
                except ValueError:
                    pass  # half written
        return runs

    # -- Gazebo ----------------------------------------------------------

    def start_gazebo(self, body):
        if self.scripts is None:
            raise ValueError("the chase suite's scripts are not found: "
                             "build with colcon build --symlink-install, "
                             "or set ROBOGAME_SCRIPTS to the repo's scripts "
                             "folder")
        if shutil.which("ros2") is None:
            raise ValueError("no ros2 on the PATH: source ROS and the "
                             "workspace, then start the app again")
        scenarios = body.get("scenarios") or list(self.scenarios)
        if (not isinstance(scenarios, list)
                or not set(scenarios) <= set(self.scenarios)):
            raise ValueError("unknown scenario")
        seconds = number(body, "seconds", 10, 300, 60, int)
        parallel = number(body, "parallel", 1, 8, 4, int)
        with self.lock:
            if self.gazebo.running():
                raise ValueError("the Gazebo chase suite is already "
                                 "running: stop it first")
            os.makedirs(self.out, exist_ok=True)
            for name in os.listdir(self.out):
                if name.endswith((".csv", ".log")) or name == ".launches":
                    os.remove(os.path.join(self.out, name))
            self.scored.clear()
            env = dict(os.environ, RUN_SECONDS=str(seconds),
                       PARALLEL=str(parallel))
            self.gazebo.start(
                ["bash", os.path.join(self.scripts, SUITE), self.out,
                 *scenarios],
                {"scenarios": scenarios, "seconds": seconds,
                 "parallel": parallel}, env)
        return self.status()

    def measure(self, path):
        """gazebo_chase_score.measure for a CSV, kept until it changes."""
        if self.scorer is None:
            spec = importlib.util.spec_from_file_location(
                "gazebo_chase_score",
                os.path.join(self.scripts, "gazebo_chase_score.py"))
            self.scorer = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.scorer)
        stat = os.stat(path)
        key = (stat.st_mtime, stat.st_size)
        if self.scored.get(path, (None,))[0] != key:
            try:
                result = self.scorer.measure(path)
            except (ValueError, KeyError, IndexError) as error:
                result = {"error": str(error)}
            self.scored[path] = (key, result)
        return self.scored[path][1]

    def results_gazebo(self):
        asked = self.gazebo.asked or {}
        running = self.gazebo.running()
        rows = []
        for name in asked.get("scenarios", []):
            base = os.path.join(self.out, name)
            row = {"scenario": name, "state": "waiting"}
            try:
                with open(base + ".log") as file:
                    done = DONE in file.read()
            except OSError:
                done = None
            if done or (done is not None and not running):
                row["state"] = "done" if done else "failed"
                if os.path.isfile(base + ".csv"):
                    row["score"] = self.measure(base + ".csv")
            elif os.path.isfile(base + ".launch.log"):
                row["state"] = "running" if running else "stopped"
            try:
                with open(base + ".launch.log", errors="replace") as file:
                    row["freed"] = file.read().count("freeing itself")
            except OSError:
                pass
            rows.append(row)
        return rows

    def end_launches(self):
        """Ends the sims the suite started, which are their own groups."""
        try:
            with open(os.path.join(self.out, ".launches")) as file:
                pids = [int(p) for p in file.read().split()]
        except (OSError, ValueError):
            return
        for sig in (signal.SIGINT, signal.SIGKILL):
            for pid in pids:
                try:
                    os.killpg(pid, sig)
                except (ProcessLookupError, PermissionError):
                    pass
            if sig == signal.SIGINT:
                time.sleep(STOP_WAIT / 2)

    # -- both ------------------------------------------------------------

    def stop(self, kind=None):
        if kind in (None, "2d"):
            self.flat.stop()
        if kind in (None, "gazebo") and self.gazebo.running():
            # The suite's shell waits on its runs; no need to be kind.
            self.gazebo.stop(signal.SIGKILL)
            if kind is None:
                self.end_launches()
            else:
                threading.Thread(target=self.end_launches,
                                 daemon=True).start()
        return self.status()

    def status(self):
        flat = self.flat.status()
        runs = self.runs_2d() if self.flat.process else []
        flat.update(runs=runs, summary=summary(runs))
        gazebo = self.gazebo.status()
        gazebo["results"] = (self.results_gazebo()
                             if self.gazebo.process else [])
        return {"2d": flat, "gazebo": gazebo, "behaviours": BEHAVIOURS,
                "scenarios": self.scenarios,
                "suite": self.scripts is not None}

    def brief(self):
        """For the status bar: what is running and how far along."""
        out = {}
        if self.flat.running():
            out["2d"] = [len(self.runs_2d()),
                         (self.flat.asked or {}).get("total", 0)]
        if self.gazebo.running():
            rows = self.results_gazebo()
            out["gazebo"] = [sum(r["state"] == "done" for r in rows),
                             len(rows)]
        return out

    def get(self, path):
        if path == "/api/chase":
            return 200, self.status()
        return None

    def post(self, path, read_body):
        try:
            if path == "/api/chase/2d":
                return 200, self.start_2d(read_body())
            if path == "/api/chase/gazebo":
                return 200, self.start_gazebo(read_body())
            if path == "/api/chase/stop":
                kind = read_body().get("kind")
                if kind not in ("2d", "gazebo"):
                    raise ValueError("kind is 2d or gazebo")
                return 200, self.stop(kind)
        except (ValueError, FileNotFoundError) as error:
            return 400, {"error": str(error)}
        return None
