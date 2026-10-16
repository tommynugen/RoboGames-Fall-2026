"""Browser editor for arena layouts.

    ros2 run robogame_sim arena_editor
    ros2 run robogame_sim arena_editor --port 8767 --no-browser

Serves a page on http://localhost:8766 where the arena's modules, 1 and 2
inch platforms, ramps, walls and the thwomp, are placed on the floor grid
as they are set up on the table, and turned to face the way they do. Layouts are saved as JSON into the
workspace's src/robogame_sim/layouts, where sim.launch.py layout:=NAME and
chase_node find them, and checked as they are edited for ramps that cannot
be driven and platforms no robot can reach. Only this computer can reach
the page.
"""

import argparse
import json
import os
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

from robogame_sim import arena_layout

PAGE = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                    "arena_editor.html")


class Handler(BaseHTTPRequestHandler):
    server_version = "arena_editor"

    def log_message(self, format, *args):
        pass

    def reply(self, status, body, kind="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(status)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def body(self):
        length = int(self.headers.get("Content-Length", 0))
        return json.loads(self.rfile.read(length) or b"{}")

    def layout_name(self):
        name = unquote(self.path[len("/api/layouts/"):])
        if not arena_layout.NAME.match(name):
            raise ValueError("names are letters, digits, - and _ only")
        return name

    def do_GET(self):
        try:
            if self.path in ("/", "/index.html"):
                with open(PAGE, "rb") as page:
                    self.reply(200, page.read(), "text/html; charset=utf-8")
            elif self.path == "/api/modules":
                self.reply(200, arena_layout.modules_info())
            elif self.path == "/api/layouts":
                self.reply(200, {"layouts": arena_layout.list_layouts(),
                                 "folder": arena_layout.layout_dirs()[0]})
            elif self.path.startswith("/api/layouts/"):
                self.reply(200, arena_layout.load(self.layout_name()))
            else:
                self.reply(404, {"error": "not found"})
        except FileNotFoundError as error:
            self.reply(404, {"error": str(error)})
        except ValueError as error:
            self.reply(400, {"error": str(error)})

    def do_PUT(self):
        try:
            if not self.path.startswith("/api/layouts/"):
                return self.reply(404, {"error": "not found"})

            layout = arena_layout.normalise(self.body())
            path = arena_layout.save(self.layout_name(), layout)
            self.reply(200, {"path": path,
                             "problems": arena_layout.check(layout)})
        except (ValueError, TypeError, KeyError) as error:
            self.reply(400, {"error": str(error)})

    def do_POST(self):
        try:
            if self.path != "/api/check":
                return self.reply(404, {"error": "not found"})

            self.reply(200, {"problems": arena_layout.check(self.body())})
        except (ValueError, TypeError, KeyError) as error:
            self.reply(400, {"error": str(error)})


def main(args=None):
    parser = argparse.ArgumentParser(description="Arena layout editor")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--no-browser", action="store_true",
                        help="do not open a browser window")
    # ros2 run can pass --ros-args, which this does not use.
    options, _ = parser.parse_known_args(args)

    server = ThreadingHTTPServer(("127.0.0.1", options.port), Handler)
    url = f"http://localhost:{options.port}/"
    print(f"Arena editor on {url}, saving layouts to "
          f"{arena_layout.layout_dirs()[0]}. Ctrl-C to stop.", flush=True)

    if not options.no_browser:
        threading.Timer(0.5, webbrowser.open, [url]).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
