"""Controller health from bluetoothctl output, with no Bluetooth."""

import time

from robogame_sim import gamepads, pairing

PAD = "AA:BB:CC:DD:EE:01"
MOUSE = "AA:BB:CC:DD:EE:02"


def info(name="DualSense Wireless Controller", icon="input-gaming",
         **fields):
    lines = [f"Device {PAD} (public)", f"\tName: {name}", f"\tIcon: {icon}"]
    lines += [f"\t{key}: {'yes' if value else 'no'}"
              for key, value in fields.items()]
    return "\n".join(lines) + "\n"


class FakeBluetooth:
    def __init__(self, infos, results=None):
        self.infos = infos  # address: info text
        self.results = results or {}
        self.calls = []

    def __call__(self, *args, timeout=None):
        self.calls.append(args)
        if args[0] == "devices":
            return "".join(f"Device {a} x\n" for a in self.infos)
        if args[0] == "info":
            return self.infos.get(args[1], "")
        if args[0] == "remove":
            self.removed = self.infos.pop(args[1], None)
            return "Device has been removed\n"
        if args[-2:] == ("scan", "on"):
            if getattr(self, "removed", None):
                self.infos[PAD] = self.removed
            return ""
        return self.results.get(args[0], f"{args[0]} successful\n")


def look(infos, pads=()):
    return pairing.Pairing(run=FakeBluetooth(infos),
                           pads=lambda: list(pads)).look()


def test_parse_info():
    fields = pairing.parse_info(info(Paired=True, Bonded=False))
    assert fields["Name"] == "DualSense Wireless Controller"
    assert fields["Paired"] is True and fields["Bonded"] is False


def test_connected_but_not_bonded_needs_pairing():
    [pad] = look({PAD: info(Paired=True, Bonded=False, Connected=True,
                            Trusted=True)})
    assert pad["state"] == "bad" and pad["action"] == "pair"
    assert "not bonded" in pad["problem"]


def test_connected_without_joystick_device():
    [pad] = look({PAD: info(Paired=True, Bonded=True, Connected=True,
                            Trusted=True)})
    assert pad["state"] == "bad" and "joystick device" in pad["problem"]


def test_ready_and_off():
    js = [("/dev/input/js0", "DualSense Wireless Controller", PAD, False)]
    [pad] = look({PAD: info(Paired=True, Bonded=True, Connected=True,
                            Trusted=True)}, js)
    assert pad["state"] == "ok" and pad["device"] == "/dev/input/js0"

    [pad] = look({PAD: info(Paired=True, Bonded=True, Connected=False,
                            Trusted=True)})
    assert pad["state"] == "warn" and pad["action"] == "connect"


def test_old_bluez_without_bonded_line_uses_paired():
    js = [("/dev/input/js0", "Wireless Controller", PAD, False)]
    [pad] = look({PAD: info(Paired=True, Connected=True, Trusted=True)}, js)
    assert pad["state"] == "ok"


def test_non_controllers_left_out():
    listed = look(
        {MOUSE: info(name="MX Master", icon="input-mouse", Connected=True)},
        [("/dev/input/js1", "keyd virtual pointer", "/dev/input/js1",
          True)])
    assert listed == []
    assert not gamepads.is_controller_name("keyd virtual pointer")
    assert gamepads.is_controller_name("DualSense Wireless Controller")


def test_wired_pad_listed_once():
    js = [("/dev/input/js0", "Sony Controller", "/dev/input/js0", False)]
    [pad] = look({}, js)
    assert pad["state"] == "ok" and pad["action"] is None


def test_pair_removes_waits_then_pairs_trusts_connects(monkeypatch):
    monkeypatch.setattr(pairing, "PAIR_WAIT", 3.0)
    bluetooth = FakeBluetooth({PAD: info(Paired=True, Bonded=False,
                                         Connected=True)})
    p = pairing.Pairing(run=bluetooth, pads=lambda: [])
    p.pair(PAD.lower())
    for _ in range(50):
        if not p.busy:
            break
        time.sleep(0.1)
    assert p.error == ""
    steps = [c[0] for c in bluetooth.calls
             if c[0] in ("remove", "pair", "trust", "connect")]
    assert steps == ["remove", "pair", "trust", "connect"]


def test_failed_step_is_reported():
    bluetooth = FakeBluetooth({PAD: info(Paired=False, Connected=False)},
                              {"connect": "Failed to connect\n"})
    p = pairing.Pairing(run=bluetooth, pads=lambda: [])
    p.connect(PAD)
    for _ in range(50):
        if not p.busy:
            break
        time.sleep(0.1)
    assert "Could not connect" in p.error


def test_bad_address_refused():
    p = pairing.Pairing(run=FakeBluetooth({}), pads=lambda: [])
    status, body = 0, None
    pairing._pairing = p
    try:
        status, body = pairing.post(None, "/api/pairing/pair",
                                    lambda: {"address": "nope"})
    finally:
        pairing._pairing = None
    assert status == 400 and "address" in body["error"]
