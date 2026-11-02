"""The RoboGame app: the arena editor and the robot console on one page.

    ros2 run robogame_sim robogame_app
    ros2 run robogame_sim robogame_app --port 8768 --no-browser --tab console

Serves http://localhost:8766 with two tabs:

- Arena editor (#editor, or #editor/NAME to open a saved layout): maps the
  modules as they are set up on the table. arena_editor.py and
  arena_editor.html.
- Robot console (#console): every robot in the arena, who drives it and
  where it goes. robot_console.py and robot_console.html.

Both tabs are live at once, so a layout saved in the editor is the one the
console draws and plans over at once (when no sim publishes its own), and
the console's Edit link opens the arena in use in the editor. Only this
computer can reach the page.

The editor's Launch buttons start the Gazebo sim on the layout open in it
and Stop stops it (sim_runner.py). Quitting the app stops the sim too.

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

from robogame_sim import arena_editor
from robogame_sim.sim_runner import SimRunner

HERE = os.path.dirname(os.path.realpath(__file__))
SHELL = os.path.join(HERE, "robogame_app.html")
# Draws the modules, for both tabs.
DRAW = os.path.join(HERE, "arena_draw.js")
TABS = ("editor", "console")
PORT = 8766

HTML = "text/html; charset=utf-8"
NO_ROS = b"""<!doctype html><meta charset="utf-8">
<body style="font: 15px system-ui, sans-serif; padding: 24px">
<h1>Robot console</h1>
<p>ROS is not sourced, so the console cannot see any robots. Source ROS,
<code>source /opt/ros/lyrical/setup.bash</code> and the workspace, and start
the app again. The arena editor works without it.</p>"""


def handler(console=None, sim=None):
    """A request handler for the app. console is the RobotConsole node, or
    None to serve the editor alone. sim is the SimRunner behind the
    editor's Launch buttons, or None for none."""
    if console is not None:
        from robogame_sim import robot_console
        console_page = robot_console.PAGE

    class Handler(BaseHTTPRequestHandler):
        server_version = "robogame_app"

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

        def file(self, path, kind):
            with open(path, "rb") as file:
                self.reply(200, file.read(), kind)

        def read_body(self):
            length = int(self.headers.get("Content-Length", 0))
            return json.loads(self.rfile.read(length) or b"{}")

        def answer(self, *routes):
            """Replies with the first route that takes the path."""
            for route in routes:
                result = route()
                if result is not None:
                    return self.reply(*result)
            self.reply(404, {"error": "not found"})

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
            if path == "/arena_view3d.js":
                return self.file(arena_editor.VIEW3D,
                                 "text/javascript; charset=utf-8")
            if path == "/api/app":
                return self.reply(200, {"console": console is not None})
            self.answer(lambda: sim and sim.get(path),
                        lambda: self.console_route("get", path),
                        lambda: arena_editor.get(path))

        def read_raw(self, limit):
            length = int(self.headers.get("Content-Length", 0))
            if length > limit:
                raise ValueError(f"that is over {limit // 2 ** 20} MB")
            return self.rfile.read(length)

        def do_PUT(self):
            self.answer(lambda: arena_editor.put(self.path, self.read_body,
                                                 self.read_raw))

        def do_POST(self):
            path = unquote(self.path)
            self.answer(
                lambda: sim and sim.post(path, self.read_body),
                lambda: self.console_route("post", path, self.read_body),
                lambda: arena_editor.post(path, self.read_body))

    return Handler


def start_console(args):
    """The robot console's node, or None without ROS."""
    try:
        import rclpy
        from robogame_sim.robot_console import RobotConsole
    except ImportError as error:
        print(f"No ROS ({error}): the robot console tab is off.", flush=True)
        return None

    rclpy.init(args=args)
    return RobotConsole()


def main(args=None, tab="editor"):
    parser = argparse.ArgumentParser(description="RoboGame app")
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--no-browser", action="store_true",
                        help="do not open a browser window")
    parser.add_argument("--tab", choices=TABS, default=tab,
                        help="the tab the browser opens on")
    # ros2 run passes --ros-args on, which rclpy takes.
    options, _ = parser.parse_known_args(args)
    url = f"http://localhost:{options.port}/#{options.tab}"

    try:
        server = ThreadingHTTPServer(("127.0.0.1", options.port), None)
    except OSError:
        # Already running, as when arena_editor and robot_console are both
        # started: one app serves both, so show it and stop.
        print(f"The RoboGame app is already on {url}.", flush=True)
        if not options.no_browser:
            webbrowser.open(url)
        return

    console = start_console(args)
    sim = SimRunner()
    server.RequestHandlerClass = handler(console, sim)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"RoboGame app on {url}, saving layouts to "
          f"{arena_editor.arena_layout.layout_dirs()[0]}. Ctrl-C to stop.",
          flush=True)

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
        if console is not None:
            import rclpy
            if rclpy.ok():
                # The firmware's 750 ms timeout stops them otherwise.
                console.stop_all()
            console.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
