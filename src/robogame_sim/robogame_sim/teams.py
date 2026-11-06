"""Team sign-up from phones, for the Gamemaster's Teams tab.

The Teams tab shows a QR code. A phone that scans it opens the sign-up
page, served on every network the computer is on (port 8767), where a
team picks its name and how many points each token colour is worth to
it, say 5 for red, 3 for blue and 1 for yellow. Each sign-up is saved,
shown on the Teams tab and published on /game/teams (std_msgs/String,
JSON, latched) for any ROS node that keeps score:

    {"tokens": ["red", "blue", "yellow"],
     "teams": [{"name": "Bolt", "points": {"red": 5, "blue": 3,
                "yellow": 1}, "updated": 1730900000.0}]}

Only the sign-up page and its one API are open to the network; the rest
of the Gamemaster stays on this computer. A phone keeps its team's id,
so scanning again edits the same team.
"""

import json
import os
import re
import secrets
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

from robogame_sim import qr

HERE = os.path.dirname(os.path.realpath(__file__))
TAB_PAGE = os.path.join(HERE, "teams.html")
SIGNUP_PAGE = os.path.join(HERE, "signup.html")
FILE_NAME = "teams.json"
TOPIC = "/game/teams"
PORT = 8767
TOKENS = ["red", "blue", "yellow"]
MAX_TEAMS = 64
MAX_NAME = 24  # characters
MAX_POINTS = 100  # a token's worth, at most
COLOUR = re.compile(r"^[a-z]{3,20}$")  # a CSS colour name
HTML = "text/html; charset=utf-8"


def teams_file():
    return os.path.join(os.path.expanduser("~/.local/share/robogame_sim"),
                        FILE_NAME)


def lan_address():
    """This computer's address on its network, the one a phone on the same
    Wi-Fi reaches it at. Sends nothing: connecting a UDP socket only picks
    the way out."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.connect(("10.255.255.255", 1))
            return probe.getsockname()[0]
        except OSError:
            return "127.0.0.1"


class Teams:
    """The teams signed up and the token colours they give points to.
    path is the JSON file they are kept in, or None to keep them in
    memory only."""

    def __init__(self, book=None, path=None, port=PORT, host=None):
        self.book = book
        self.path = path
        self.port = port
        self.host = host
        self.lock = threading.Lock()
        self.tokens = list(TOKENS)
        self.teams = {}  # id: {"name", "points", "updated"}
        self.publish = None
        self.server = None
        self.error = None
        self.load()

    def say(self, text, level="info"):
        if self.book is not None:
            self.book.add("teams", text, level)

    # Saving

    def load(self):
        if not self.path or not os.path.exists(self.path):
            return
        try:
            with open(self.path) as file:
                saved = json.load(file)
            self.tokens = [t for t in saved.get("tokens", TOKENS)
                           if isinstance(t, str) and COLOUR.match(t)]
            self.teams = {key: team for key, team in
                          saved.get("teams", {}).items()
                          if isinstance(team, dict) and "name" in team}
        except (OSError, ValueError, AttributeError) as error:
            self.say(f"Could not read {self.path}: {error}", "warn")

    def save(self):
        """Saves and publishes. Called with the lock held."""
        if self.path:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with open(self.path, "w") as file:
                json.dump({"tokens": self.tokens, "teams": self.teams},
                          file, indent=1)
        if self.publish is not None:
            self.publish(json.dumps(self.message()))

    def message(self):
        return {"tokens": list(self.tokens),
                "teams": [{"name": team["name"],
                           "points": dict(team["points"]),
                           "updated": team["updated"]}
                          for team in self.listed()]}

    def listed(self):
        return sorted(self.teams.values(), key=lambda t: t["updated"])

    # ROS side

    def attach(self, node):
        """Publishes /game/teams on node, latched, now and on each
        change."""
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
        publisher = node.create_publisher(String, TOPIC, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publish = lambda data: publisher.publish(String(data=data))
        with self.lock:
            self.publish(json.dumps(self.message()))

    # Changes

    def sign_up(self, body):
        """A team from a phone: {"id" (to edit it, or none for a new
        team), "name", "points": {colour: points}}. Returns the team with
        its id."""
        name = " ".join(str(body.get("name", "")).split())
        if not name:
            raise ValueError("give the team a name")
        if len(name) > MAX_NAME:
            raise ValueError(f"keep the name to {MAX_NAME} letters")
        given = body.get("points")
        if not isinstance(given, dict):
            raise ValueError("points should be one number per colour")
        with self.lock:
            points = {}
            for colour in self.tokens:
                value = given.get(colour, 0)
                if isinstance(value, str) and value.strip() == "":
                    value = 0
                try:
                    value = int(value)
                except (TypeError, ValueError):
                    raise ValueError(f"{colour} should be a whole number")
                if not 0 <= value <= MAX_POINTS:
                    raise ValueError(
                        f"{colour} should be 0 to {MAX_POINTS} points")
                points[colour] = value
            key = body.get("id")
            if key not in self.teams:
                key = None
            for other_key, other in self.teams.items():
                if other_key != key and \
                        other["name"].casefold() == name.casefold():
                    raise ValueError(f"{other['name']} is taken; "
                                     "pick another name")
            if key is None:
                if len(self.teams) >= MAX_TEAMS:
                    raise ValueError("the game is full")
                key = secrets.token_hex(4)
                verb = "signed up"
            else:
                verb = "changed"
            team = {"name": name, "points": points, "updated": time.time()}
            self.teams[key] = team
            self.save()
        self.say(f"Team {name} {verb}: " + ", ".join(
            f"{colour} {value}" for colour, value in points.items()))
        return {"id": key, **team}

    def set_tokens(self, tokens):
        if isinstance(tokens, str):
            tokens = re.split(r"[\s,]+", tokens)
        if not isinstance(tokens, list):
            raise ValueError("tokens should be a list of colours")
        clean = []
        for token in tokens:
            token = str(token).strip().lower()
            if not token:
                continue
            if not COLOUR.match(token):
                raise ValueError(f"{token!r} is not a colour name")
            if token not in clean:
                clean.append(token)
        if not clean:
            raise ValueError("give at least one token colour")
        with self.lock:
            self.tokens = clean
            for team in self.teams.values():
                team["points"] = {c: team["points"].get(c, 0) for c in clean}
            self.save()
        self.say("Token colours: " + ", ".join(clean))

    def remove(self, key):
        with self.lock:
            team = self.teams.pop(key, None)
            if team is None:
                raise ValueError("no such team")
            self.save()
        self.say(f"Team {team['name']} removed.")

    def clear(self):
        with self.lock:
            self.teams.clear()
            self.save()
        self.say("All teams removed.")

    # The Gamemaster's routes, for the Teams tab

    def url(self):
        return f"http://{self.host or lan_address()}:{self.port}/"

    def state(self):
        with self.lock:
            return {"url": self.url(), "tokens": list(self.tokens),
                    "teams": [{"id": key, **team} for key, team in
                              sorted(self.teams.items(),
                                     key=lambda kv: kv[1]["updated"])],
                    "serving": self.server is not None, "error": self.error,
                    "topic": TOPIC}

    def get(self, path):
        if path == "/api/teams":
            return 200, self.state()
        if path == "/api/teams/qr.svg":
            return 200, qr.svg(self.url()).encode(), "image/svg+xml"
        return None

    def post(self, path, read_body):
        try:
            if path == "/api/teams/tokens":
                self.set_tokens(read_body().get("tokens"))
                return 200, self.state()
            if path == "/api/teams/remove":
                self.remove(read_body().get("id"))
                return 200, self.state()
            if path == "/api/teams/clear":
                self.clear()
                return 200, self.state()
        except ValueError as error:
            return 400, {"error": str(error)}
        return None

    # The sign-up server, open to the network

    def serve(self):
        """Starts the sign-up page on every network, port self.port."""
        try:
            self.server = ThreadingHTTPServer(("0.0.0.0", self.port),
                                              signup_handler(self))
        except OSError as error:
            self.error = f"port {self.port} is busy: {error.strerror}"
            self.say(f"Phone sign-up is off: {self.error}", "error")
            return
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever,
                         daemon=True).start()
        self.say(f"Phone sign-up on {self.url()}")

    def stop(self):
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None


def signup_handler(teams):
    """The phones' side: the sign-up page and /api/signup, nothing else."""

    class Handler(BaseHTTPRequestHandler):
        server_version = "robogame_signup"

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

        def do_GET(self):
            url = urlsplit(self.path)
            if url.path in ("/", "/index.html"):
                with open(SIGNUP_PAGE, "rb") as file:
                    return self.reply(200, file.read(), HTML)
            if url.path == "/api/signup":
                key = parse_qs(url.query).get("id", [None])[0]
                with teams.lock:
                    team = teams.teams.get(key)
                    return self.reply(200, {
                        "tokens": list(teams.tokens),
                        "max_points": MAX_POINTS, "max_name": MAX_NAME,
                        "team": {"id": key, **team} if team else None})
            self.reply(404, {"error": "not found"})

        def do_POST(self):
            if urlsplit(self.path).path != "/api/signup":
                return self.reply(404, {"error": "not found"})
            try:
                length = int(self.headers.get("Content-Length", 0))
                if length > 4096:
                    raise ValueError("that is too long")
                body = json.loads(self.rfile.read(length) or b"{}")
                if not isinstance(body, dict):
                    raise ValueError("send a JSON object")
                self.reply(200, teams.sign_up(body))
            except ValueError as error:
                self.reply(400, {"error": str(error)})

    return Handler
