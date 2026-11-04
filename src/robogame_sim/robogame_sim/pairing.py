"""Controller health for the robot console: what Bluetooth thinks of each
controller, and whether Linux made a joystick device for it.

A controller can show as connected in bluetoothctl and still drive
nothing: when it is connected but not bonded, the robot PC drops its input
(the "!bonded" line in the system log), and with no joydev driver there is
no /dev/input/jsN to read. This puts those problems next to the
controllers on the console page, with a Pair button that removes, pairs,
trusts and connects the controller again.

Routes, served through robot_console's get and post:

- GET /api/pairing: {"controllers": [{"address", "name", "state",
  "problem", "action", "device"}, ...], "scanning", "busy", "error"}.
  state is ok, warn or bad; action is pair, connect or None.
- POST /api/pairing/scan: looks for new controllers for SCAN_TIME s.
- POST /api/pairing/pair {"address"}: pairs that controller again.
- POST /api/pairing/connect {"address"}: connects a paired controller.

Everything goes through bluetoothctl, run on the machine the Gamemaster
runs on, which is the one the controllers are connected to.
"""

import re
import shutil
import subprocess
import threading
import time

from robogame_sim import gamepads

SCAN_TIME = 15.0  # s, a scan for new controllers
PAIR_WAIT = 20.0  # s, for a removed controller to be seen again
CACHE = 2.0  # s, how long one look at bluetoothctl is reused
RUN_TIMEOUT = 30.0  # s, for one bluetoothctl command

ADDRESS = re.compile(r"^([0-9A-F]{2}:){5}[0-9A-F]{2}$")
# Names of game controllers that do not say input-gaming in bluetoothctl.
CONTROLLER_NAMES = re.compile(
    r"wireless controller|dualsense|dualshock|xbox|pro controller|gamepad|"
    r"joy-con|8bitdo", re.IGNORECASE)


def run_bluetoothctl(*args, timeout=RUN_TIMEOUT):
    """bluetoothctl's output for one command, or "" when it fails."""
    try:
        done = subprocess.run(["bluetoothctl", *args], capture_output=True,
                              text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return ""
    return done.stdout


def parse_devices(text):
    """[(address, name)] from bluetoothctl devices."""
    found = []
    for line in text.splitlines():
        match = re.match(r"^\s*Device\s+(\S+)\s*(.*)$", line)
        if match and ADDRESS.match(match.group(1).upper()):
            found.append((match.group(1).upper(), match.group(2).strip()))
    return found


def parse_info(text):
    """The fields of bluetoothctl info, as {"Name": ..., "Paired": True},
    yes and no turned into True and False."""
    fields = {}
    for line in text.splitlines():
        match = re.match(r"^\s+([A-Za-z ]+):\s*(.*)$", line)
        if not match or match.group(1) in fields:
            continue
        value = match.group(2).strip()
        fields[match.group(1)] = {"yes": True, "no": False}.get(value, value)
    return fields


def is_controller(name, info):
    if not gamepads.is_controller_name(name):
        return False
    return (info.get("Icon") == "input-gaming"
            or bool(CONTROLLER_NAMES.search(name or "")))


def health(address, name, info, device):
    """(state, problem, action) for one Bluetooth controller. info is its
    parse_info, device its /dev/input/jsN or None."""
    paired = info.get("Paired") is True
    # bluez before 5.66 has no Bonded line; Paired is the nearest then.
    bonded = info.get("Bonded", paired) is True
    connected = info.get("Connected") is True
    trusted = info.get("Trusted") is True

    if connected and not bonded:
        return ("bad", "Connected but not bonded, so its input is dropped. "
                "Hold Create and PS until the light flashes, then Pair.",
                "pair")
    if connected and device is None:
        return ("bad", "Connected but there is no joystick device for it. "
                "Load the driver (sudo modprobe joydev) or pair it again.",
                "pair")
    if connected and not trusted:
        return ("warn", f"Works on {device}, but is not trusted, so it will "
                "not reconnect by itself. Pair fixes that.", "pair")
    if connected:
        return ("ok", f"Ready on {device}.", None)
    if paired:
        return ("warn", "Paired but off. Press its PS button, or Connect.",
                "connect")
    return ("warn", "Seen but not paired. Hold Create and PS until the "
            "light flashes, then Pair.", "pair")


class Pairing:
    """Looks at bluetoothctl and the js devices, and runs the Pair and
    Connect buttons one at a time in a thread. run and pads stand in for
    bluetoothctl and gamepads.find_gamepads in tests."""

    def __init__(self, run=run_bluetoothctl, pads=gamepads.find_gamepads,
                 log=None):
        self.run = run
        self.pads = pads
        self.log = log or (lambda text, level="info": None)
        self.lock = threading.Lock()
        self.busy = {}  # address: what is being done to it
        self.scan_until = 0.0
        self.error = ""
        self.cached = None
        self.cached_at = 0.0

    def available(self):
        return self.run is not run_bluetoothctl or bool(
            shutil.which("bluetoothctl"))

    def look(self):
        """The controllers, Bluetooth ones first, then those with a js
        device and no Bluetooth (USB, virtual)."""
        devices = {}
        for device, name, address, virtual in self.pads():
            devices[address] = (device, name, virtual)

        listed = []
        seen = set()
        if self.available():
            for address, name in parse_devices(self.run("devices")):
                info = parse_info(self.run("info", address))
                name = info.get("Name") or name
                if not is_controller(name, info):
                    continue
                seen.add(address)
                device = devices.get(address, (None,))[0]
                state, problem, action = health(address, name, info, device)
                listed.append({"address": address, "name": name,
                               "state": state, "problem": problem,
                               "action": action, "device": device})

        for address, (device, name, virtual) in sorted(devices.items()):
            if address in seen or not gamepads.is_controller_name(name):
                continue
            listed.append({
                "address": address, "name": name, "state": "ok",
                "problem": (f"Virtual pad on {device}, made by software "
                            "such as Steam or Sunshine." if virtual else
                            f"Ready on {device}."),
                "action": None, "device": device})
        return listed

    def state(self):
        now = time.monotonic()
        if self.cached is None or now - self.cached_at > CACHE:
            self.cached = self.look()
            self.cached_at = now
        with self.lock:
            controllers = [dict(c, busy=self.busy.get(c["address"]))
                           for c in self.cached]
            return {"controllers": controllers,
                    "available": self.available(),
                    "scanning": now < self.scan_until,
                    "busy": bool(self.busy),
                    "error": self.error}

    # -- buttons ---------------------------------------------------------

    def start(self, address, what, work):
        if not self.available():
            raise ValueError("bluetoothctl is not installed")
        with self.lock:
            if address in self.busy:
                raise ValueError(f"already busy with {address}")
            self.busy[address] = what
            self.error = ""

        def go():
            try:
                work()
            except Exception as error:  # shown on the page, not lost
                with self.lock:
                    self.error = str(error)
                self.log(str(error), "error")
            finally:
                with self.lock:
                    self.busy.pop(address, None)
                self.cached = None

        threading.Thread(target=go, daemon=True).start()

    def scan(self):
        def work():
            self.scan_until = time.monotonic() + SCAN_TIME
            self.log(f"Looking for controllers for {SCAN_TIME:.0f} s")
            self.run("--timeout", str(int(SCAN_TIME)), "scan", "on",
                     timeout=SCAN_TIME + 5)
        self.start("scan", "scanning", work)

    def checked(self, address):
        address = str(address or "").upper()
        if not ADDRESS.match(address):
            raise ValueError("pair needs a Bluetooth address")
        return address

    def pair(self, address):
        address = self.checked(address)
        self.start(address, "pairing", lambda: self.pair_now(address))

    def connect(self, address):
        address = self.checked(address)

        def work():
            self.log(f"Connecting {address}")
            if not self.succeeded(self.run("connect", address)):
                raise RuntimeError(f"Could not connect {address}. Press "
                                   "its PS button and try again.")
        self.start(address, "connecting", work)

    def pair_now(self, address):
        """Forgets the controller, waits to see it in pairing mode, then
        pairs, trusts and connects it."""
        self.log(f"Pairing {address} again: removing it first")
        self.run("remove", address)
        scanner = threading.Thread(target=self.run, args=(
            "--timeout", str(int(PAIR_WAIT)), "scan", "on"),
            kwargs={"timeout": PAIR_WAIT + 5}, daemon=True)
        self.scan_until = time.monotonic() + PAIR_WAIT
        scanner.start()

        give_up = time.monotonic() + PAIR_WAIT
        while not any(a == address for a, _ in
                      parse_devices(self.run("devices"))):
            if time.monotonic() > give_up:
                raise RuntimeError(
                    f"{address} was not seen. Hold Create and PS until "
                    "the light flashes fast, then Pair again.")
            time.sleep(1.0)

        for step in ("pair", "trust", "connect"):
            self.log(f"bluetoothctl {step} {address}")
            if not self.succeeded(self.run(step, address)):
                raise RuntimeError(f"bluetoothctl {step} {address} failed")
        self.log(f"Paired {address}")

    @staticmethod
    def succeeded(output):
        return bool(re.search(r"successful|succeeded|Already", output)) \
            and not re.search(r"Failed|not available", output)


_pairing = None


def pairing_for(console):
    global _pairing
    if _pairing is None:
        logger = console.get_logger() if console is not None else None

        def log(text, level="info"):
            if logger is not None:
                (logger.error if level == "error" else logger.info)(text)
        _pairing = Pairing(log=log)
    return _pairing


def get(console, path):
    if path == "/api/pairing":
        return 200, pairing_for(console).state()
    return None


def post(console, path, read_body):
    pairing = pairing_for(console)
    if path == "/api/pairing/scan":
        call = (pairing.scan,)
    elif path in ("/api/pairing/pair", "/api/pairing/connect"):
        try:
            asked = read_body()
        except ValueError:
            return 400, {"error": "bad JSON"}
        call = (pairing.pair if path.endswith("pair") else pairing.connect,
                asked.get("address"))
    else:
        return None
    try:
        call[0](*call[1:])
    except ValueError as error:
        return 400, {"error": str(error)}
    return 200, {"ok": True}
