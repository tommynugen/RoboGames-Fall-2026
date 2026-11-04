"""The Gazebo window's camera from the RoboGame app: the views fpv_toggle
gives the 1 to 4 keys and the Camera view card.

Talks to fpv_toggle over gz-transport as the card does: asks for a view on
/robogame/view with "VIEW ROBOT", such as "top robot_2" or "fpv robot_1",
and reads /robogame/view/state, which fpv_toggle sends once a second and
on every change: the view, the robot, then every robot with a camera.
With no state in the last STALE seconds there is no Gazebo window to
move: the sim is not running, or runs with gui:=false.

Routes, served by gamemaster.py:

- GET /api/sim/view: {"available", "view", "robot", "robots", "views",
  "error"}
- POST /api/sim/view {"view": "top" | "front" | "front_right" | "fpv",
  "robot": "robot_1"}: moves the camera there

The sim's partition (launch option partition) is passed in, so a sim run
in its own partition is reached in it.
"""

import threading
import time

VIEWS = ("top", "front", "front_right", "fpv")
VIEW_TOPIC = "/robogame/view"
STATE_TOPIC = "/robogame/view/state"
STALE = 3.0  # s with no state before the window counts as gone
WAIT = 1.5  # s to wait for fpv_toggle's answer, which comes once a second


class SimCamera:
    def __init__(self, partition=lambda: ""):
        self.partition = partition  # called for the sim's partition now
        self.node = None
        self.node_partition = None
        self.publisher = None
        self.state = None
        self.seen = 0.0
        self.error = ""
        # Reentrant: gz-transport hands a subscriber in this same process
        # the message inside publish(), so on_state can run under look().
        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)

    def connect(self):
        """A gz-transport node in the sim's partition, made again when the
        partition changes. False, with self.error saying why, when there
        are no gz-transport Python bindings."""
        partition = self.partition() or ""
        if self.node is not None and partition == self.node_partition:
            return True
        try:
            from gz.msgs.stringmsg_pb2 import StringMsg  # noqa: F401
            from gz.msgs.stringmsg_v_pb2 import StringMsg_V
            from gz.transport import Node
        except ImportError as error:
            self.error = f"no gz-transport Python bindings ({error})"
            return False
        try:
            from gz.transport import NodeOptions
            options = NodeOptions()
            if partition:
                options.partition = partition
            node = Node(options)
        except (ImportError, TypeError, AttributeError):
            node = Node()  # an older binding: the app's own GZ_PARTITION
        from gz.msgs.stringmsg_pb2 import StringMsg
        self.publisher = node.advertise(VIEW_TOPIC, StringMsg)
        node.subscribe(StringMsg_V, STATE_TOPIC, self.on_state)
        self.node, self.node_partition = node, partition
        self.state, self.seen, self.error = None, 0.0, ""
        return True

    def on_state(self, message):
        data = list(message.data)
        with self.lock:
            self.state = {"view": data[0] if data else "",
                          "robot": data[1] if len(data) > 1 else "",
                          "robots": data[2:]}
            self.seen = time.monotonic()
            self.changed.notify_all()

    def wait(self, done):
        """Waits up to WAIT for a state where done(state) holds. Called
        with the lock held, which waiting lets go of."""
        self.changed.wait_for(lambda: self.state is not None
                              and done(self.state), WAIT)

    def status(self):
        with self.lock:
            new = self.node is None or self.partition() != self.node_partition
            connected = self.connect()
            if connected and new:
                # A new node has heard nothing yet: give fpv_toggle a
                # second to say there is a window before saying there isn't.
                self.wait(lambda state: True)
            fresh = time.monotonic() - self.seen < STALE
            state = self.state if fresh else None
        reply = {"available": state is not None, "views": list(VIEWS),
                 "view": "", "robot": "", "robots": [], "error": ""}
        if state:
            reply.update(state)
        elif not connected:
            reply["error"] = self.error
        else:
            reply["error"] = ("no Gazebo window: launch the sim with the "
                              "window on")
        return reply

    def look(self, view, robot=""):
        if view not in VIEWS:
            raise ValueError("view takes " + ", ".join(VIEWS))
        robot = str(robot or "").strip()
        if robot and not robot.replace("_", "").isalnum():
            raise ValueError(f"no robot {robot!r}")
        with self.lock:
            if not self.connect():
                raise ValueError(self.error)
            from gz.msgs.stringmsg_pb2 import StringMsg
            message = StringMsg()
            message.data = f"{view} {robot}".strip()
            asked = time.monotonic()
            self.publisher.publish(message)
            # Answer with the view it moved to, not the one before.
            self.wait(lambda state: self.seen >= asked
                      and state["view"] == view
                      and (not robot or state["robot"] == robot))
        return self.status()

    def get(self, path):
        if path == "/api/sim/view":
            return 200, self.status()
        return None

    def post(self, path, read_body):
        if path != "/api/sim/view":
            return None
        try:
            body = read_body()
            return 200, self.look(body.get("view"), body.get("robot"))
        except ValueError as error:
            return 400, {"error": str(error)}
