"""The RoboGame Gamemaster: the arena editor and the robot console on one page.

    ros2 run robogame_sim robogame_gamemaster
    ros2 run robogame_sim robogame_gamemaster --port 8768 --tab console

Serves http://localhost:8766 with five tabs:

- Arena editor (#editor, or #editor/NAME to open a saved layout): maps the
  modules as they are set up on the table. arena_editor.py and
  arena_editor.html.
- Robot console (#console): every robot in the arena, who drives it and
  where it goes. robot_console.py and robot_console.html.
- Vision (#vision): the arena camera live, the AprilTags it finds, where
  each robot is and the detector's settings. vision.py and vision.html.
- Logs (#logs): the log lines of the editor, the robot console, the
  controller nodes, the sim and the app, each on its own tab and all
  together on All. app_log.py and app_log.html.
- Upkeep (#upkeep): the repo's jobs, the Onshape export, the
  arena_perception copy and the colcon build, with their output.
  upkeep.py and upkeep.html.

Right of the tabs a status bar shows, every 2 s from GET /api/app,
whether ROS is on, the sim's state, the BLE bridge's, how many robots
the console sees and the arena in use; each opens the tab that deals with it. The Logs tab
counts the errors not yet looked at, and a new one pops up in a corner
while another tab shows. Alt+1 to Alt+5 switch tabs, and a bare URL
opens the tab shown last.

Both tabs are live at once, so a layout saved in the editor is the one the
console draws and plans over at once (when no sim publishes its own), and
the console's Edit link opens the arena in use in the editor. Only this
computer can reach the page.

The editor's Launch buttons start the Gazebo sim on the layout open in it
and Stop stops it (sim_runner.py), with every sim.launch.py option from
its Launch options panel, and its Gazebo camera panel moves the Gazebo
window's view (sim_camera.py). Quitting the app stops the sim too.

Without ROS (no rclpy), as on a laptop only mapping the table, the editor
still works and the console tab says ROS is not sourced.
"""

import argparse
import json
import os
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

from robogame_sim import app_log, arena_editor, vision as vision_tab
from robogame_sim.ble_bridge import BridgeRunner
from robogame_sim.sim_runner import SimRunner
from robogame_sim.upkeep import UpkeepRunner

HERE = os.path.dirname(os.path.realpath(__file__))
SHELL = os.path.join(HERE, "gamemaster.html")
# Draws the modules, for both tabs.
DRAW = os.path.join(HERE, "arena_draw.js")
DIALOGS = os.path.join(HERE, "dialogs.js")
LOGS_PAGE = os.path.join(HERE, "app_log.html")
VISION_PAGE = os.path.join(HERE, vision_tab.PAGE_NAME)
UPKEEP_PAGE = os.path.join(HERE, "upkeep.html")
TABS = ("editor", "console", "vision", "logs", "upkeep")
# Changes the editor makes as it is used, not worth a log line each.
QUIET = ("/api/check",)
PORT = 8766

HTML = "text/html; charset=utf-8"
NO_ROS = b"""<!doctype html><meta charset="utf-8">
<body style="font: 15px system-ui, sans-serif; padding: 24px">
<h1>Robot console</h1>
<p>ROS is not sourced, so the console cannot see any robots. Source ROS,
<code>source /opt/ros/lyrical/setup.bash</code> and the workspace, and start
the app again. The arena editor works without it.</p>"""


def handler(console=None, sim=None, book=None, bridge=None, vision=None,
            upkeep=None):
    """A request handler for the app. console is the RobotConsole node, or
    None to serve the editor alone. sim is the SimRunner behind the
    editor's Launch buttons, or None for none. book is the LogBook the
    Logs tab shows, which gets a line for each change asked. bridge
    is the BridgeRunner behind the console's Bridge buttons, or None.
    vision is the Vision tab's camera and tags. upkeep is the
    UpkeepRunner behind the Upkeep tab, or None."""
    book = book or app_log.LogBook()
    vision = vision or vision_tab.Vision(book)
    if console is not None:
        from robogame_sim import robot_console
        console_page = robot_console.PAGE

    class Handler(BaseHTTPRequestHandler):
        server_version = "robogame_gamemaster"

        def log_message(self, format, *args):
            pass

        def reply(self, status, body, kind="application/json"):
            data = body if isinstance(body, bytes) else \
                json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
            self.status, self.body = status, body

        def file(self, path, kind):
            with open(path, "rb") as file:
                self.reply(200, file.read(), kind)

        def read_body(self):
            length = int(self.headers.get("Content-Length", 0))
            return json.loads(self.rfile.read(length) or b"{}")

        def answer(self, *routes):
            """Replies with the first route that takes the path. routes
            are (source, route): source names the Logs tab the
            request's log line goes to."""
            for source, route in routes:
                result = route()
                if result is not None:
                    self.reply(*result)
                    return source
            self.reply(404, {"error": "not found"})
            return "app"

        def note(self, source, path):
            """A log line for a change asked of the app."""
            if path in QUIET and self.status < 400:
                return
            error = isinstance(self.body, dict) and self.body.get("error")
            book.add(source, f"{self.command} {path}: {self.status}"
                     + (f" {error}" if error else ""),
                     "error" if self.status >= 400 else "info")

        def console_route(self, name, *args):
            if console is None:
                return None
            from robogame_sim import robot_console
            return getattr(robot_console, name)(console, *args)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                return self.file(SHELL, HTML)
            if path == "/editor":
                return self.file(arena_editor.PAGE, HTML)
            if path == "/console":
                if console is None:
                    return self.reply(200, NO_ROS, HTML)
                return self.file(console_page, HTML)
            if path == "/arena_draw.js":
                return self.file(DRAW, "text/javascript; charset=utf-8")
            if path == "/dialogs.js":
                return self.file(DIALOGS, "text/javascript; charset=utf-8")
            if path == "/arena_view3d.js":
                return self.file(arena_editor.VIEW3D,
                                 "text/javascript; charset=utf-8")
            if path == "/logs":
                return self.file(LOGS_PAGE, HTML)
            if path == "/vision":
                return self.file(VISION_PAGE, HTML)
            if path == "/upkeep":
                return self.file(UPKEEP_PAGE, HTML)
            if path == "/vision/stream.mjpg":
                return self.stream()
            if path == "/api/app":
                return self.reply(200, status(console, sim, book, bridge,
                                              upkeep))
            source = self.answer(
                ("app", lambda: book.get(self.path)),
                ("sim", lambda: sim and sim.get(path)),
                ("bridge", lambda: bridge and bridge.get(path)),
                ("upkeep", lambda: upkeep and upkeep.get(path)),
                ("vision", lambda: vision.get(path)),
                ("console", lambda: self.console_route("get", path)),
                ("editor", lambda: arena_editor.get(path)))
            if self.status >= 400:
                self.note(source, path)

        def stream(self):
            """The Vision tab's picture, live, as MJPEG."""
            if not vision.running.is_set():
                return self.reply(503, {"error": "the camera is not on"})
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; "
                             f"boundary={vision_tab.BOUNDARY}")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                vision.stream(self.wfile.write)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def read_raw(self, limit):
            length = int(self.headers.get("Content-Length", 0))
            if length > limit:
                raise ValueError(f"that is over {limit // 2 ** 20} MB")
            return self.rfile.read(length)

        def do_PUT(self):
            self.note(self.answer(
                ("editor", lambda: arena_editor.put(
                    self.path, self.read_body, self.read_raw))),
                unquote(self.path.split("?")[0]))

        def do_POST(self):
            path = unquote(self.path)
            self.note(self.answer(
                ("sim", lambda: sim and sim.post(path, self.read_body)),
                ("bridge", lambda: bridge and bridge.post(
                    path, self.read_body)),
                ("upkeep", lambda: upkeep and upkeep.post(
                    path, self.read_body)),
                ("vision", lambda: vision.post(path, self.read_body)),
                ("console", lambda: self.console_route(
                    "post", path, self.read_body)),
                ("editor", lambda: arena_editor.post(path, self.read_body))),
                path)
            if path == "/api/sim" and self.status == 200 and sim:
                # A new run: its log file starts again.
                book.follow("sim", sim.log, 0)
            if path == "/api/upkeep" and self.status == 200 and upkeep:
                book.follow("upkeep", upkeep.log, 0)

    return Handler


def status(console, sim, book, bridge=None, upkeep=None):
    """What the shell's status bar shows: whether ROS is on, the sim's
    state, the robots and arena the console sees, the upkeep job's and
    the newest error line."""
    reply = {"console": console is not None, "sim": None, "robots": None,
             "arena": None, "bridge": None, "upkeep": None,
             "logs": book.summary()}
    if upkeep is not None and upkeep.job:
        reply["upkeep"] = {"job": upkeep.job, "running": upkeep.running(),
                           "stopped": upkeep.stopped,
                           "exit": upkeep.process.poll()}
    if bridge is not None:
        state = bridge.status()
        reply["bridge"] = {"state": state["state"],
                           "robots": len(state["robots"])}
    if sim is not None:
        state = sim.status()
        reply["sim"] = {key: state.get(key)
                        for key in ("running", "ready", "layout", "exit",
                                    "stopped")}
    if console is not None:
        snapshot = console.state() or {}
        reply["robots"] = len(snapshot.get("robots", []))
        reply["arena"] = (snapshot.get("arena") or {}).get("name")
    return reply


def say(book, text, level="info"):
    """Prints text and puts it on the Logs tab's app log."""
    print(text, flush=True)
    book.add("app", text, level)


def start_console(args, book):
    """The robot console's node, or None without ROS."""
    try:
        import rclpy
        from robogame_sim.robot_console import RobotConsole
    except ImportError as error:
        say(book, f"No ROS ({error}): the robot console tab is off.",
            "warn")
        return None

    rclpy.init(args=args)
    node = RobotConsole()
    try:
        app_log.watch_rosout(node, book)
    except ImportError as error:
        say(book, f"No ROS log ({error}).", "warn")
    return node


def main(args=None):
    parser = argparse.ArgumentParser(description="RoboGame Gamemaster")
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--no-browser", action="store_true",
                        help="do not open a browser window")
    parser.add_argument("--tab", choices=TABS, default="editor",
                        help="the tab the browser opens on")
    # ros2 run passes --ros-args on, which rclpy takes.
    options, _ = parser.parse_known_args(args)
    url = f"http://localhost:{options.port}/#{options.tab}"

    try:
        server = ThreadingHTTPServer(("127.0.0.1", options.port), None)
    except OSError:
        # Already running: show it and stop.
        print(f"The RoboGame Gamemaster is already on {url}.", flush=True)
        if not options.no_browser:
            webbrowser.open(url)
        return

    book = app_log.LogBook()
    console = start_console(args, book)
    sim = SimRunner()
    book.follow("sim", sim.log)
    bridge = BridgeRunner(book=book)
    if console is not None:
        console.bridge = bridge
    vision = vision_tab.Vision(book)
    if console is not None:
        vision.attach(console)
    upkeep = UpkeepRunner()
    server.RequestHandlerClass = handler(console, sim, book, bridge, vision,
                                         upkeep)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    say(book, f"RoboGame Gamemaster on {url}, saving layouts to "
        f"{arena_editor.arena_layout.layout_dirs()[0]}. Ctrl-C to stop.")

    if not options.no_browser:
        threading.Timer(0.5, webbrowser.open, [url]).start()

    try:
        if console is None:
            threading.Event().wait()
        else:
            import rclpy
            from rclpy.executors import ExternalShutdownException
            try:
                rclpy.spin(console)
            except ExternalShutdownException:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        sim.stop()
        bridge.stop()
        upkeep.stop()
        vision.stop()
        if console is not None:
            import rclpy
            if rclpy.ok():
                # The firmware's 750 ms timeout stops them otherwise.
                console.stop_all()
            console.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
