"""Moves the Gazebo window between views over the arena and a robot's FPV
camera.

    ros2 run robogame_sim fpv_toggle --robots 1,2

With the 3D view focused, 1 looks straight down on the arena, 2 over its
front (-y) edge, 3 over its front right corner, and 4 toggles FPV, going
back to the last of those views. F1 to F9 look through that robot's
camera instead, robots spawned since too. Escape also leaves FPV, but
stays where it is rather than going back over the arena. The FPV view
sits where robot_sdf mounts the camera, on the front of the chassis, and
turns with the chassis only, so the arm moving does not move it.

The Camera view card in the window's side panel (robogame_gui) does the
same from two drop downs. It asks for a view on /robogame/view with the
view's name and a robot, "top robot_2" or "fpv robot_1", and fills its
drop downs from /robogame/view/state, which this publishes on every
change and once a second: the view, the robot, then every robot.

Talks to the Gazebo GUI over gz-transport, not ROS: it reads the keys the
KeyPublisher GUI plugin publishes on /keyboard/keypress, and moves the GUI
camera with the CameraTracking plugin's /gui/track and /gui/move_to/pose.
sim.launch.py starts it, and adds KeyPublisher, whenever the window is up.
With --no-fpv (fpv:=false) the robots have no camera, and only the views
over the arena work.
"""

import argparse
import math
import queue
import signal
import threading
import time

from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.cameratrack_pb2 import CameraTrack
from gz.msgs.empty_pb2 import Empty
from gz.msgs.gui_camera_pb2 import GUICamera
from gz.msgs.int32_pb2 import Int32
from gz.msgs.scene_pb2 import Scene
from gz.msgs.stringmsg_pb2 import StringMsg
from gz.msgs.stringmsg_v_pb2 import StringMsg_V
from gz.transport import Node

from robogame_sim.robot_sdf import FPV_CAMERA

# Qt key codes, as KeyPublisher sends them.
KEY_1 = 0x31
KEY_FPV = 0x34  # 4
KEY_F1 = 0x01000030
KEY_ESCAPE = 0x01000000

NO_TARGET = "fpv_off"
FPV = "fpv"

# Views over the arena, x y z roll pitch yaw. The arena is 1.22 m across,
# its middle at the origin. Straight down is a hair off 90 degrees, so the
# view still knows which way is up: the front edge at the bottom.
VIEWS = {
    "top": (0.0, 0.0, 2.2, 0.0, math.pi / 2 - 0.001, math.pi / 2),
    "front": (0.0, -1.1, 1.0, 0.0, 0.75, math.pi / 2),
    "front_right": (0.85, -0.85, 1.05, 0.0, 0.72, 3 * math.pi / 4),
}
VIEW_KEYS = {KEY_1 + index: name for index, name in enumerate(VIEWS)}

# The view the window starts on, and that the launch puts in gui.config.
START_VIEW = "front"
OVERVIEW = VIEWS[START_VIEW]

VIEW_TOPIC = "/robogame/view"
STATE_TOPIC = "/robogame/view/state"


def set_vector(vector, values):
    vector.x, vector.y, vector.z = values


def fpv_track(robot):
    """CameraTrack that holds the GUI camera on robot's FPV camera."""
    x, y, z, _ = FPV_CAMERA
    message = CameraTrack()
    message.track_mode = CameraTrack.FOLLOW_LOOK_AT
    # The follow offset is in the robot's frame. The track offset is not
    # used: the camera looks at the track target itself, so it looks at
    # the sight robot_sdf puts on the chassis, out along the camera's line.
    message.follow_target.name = robot
    set_vector(message.follow_offset, (x, y, z))
    message.track_target.name = f"{robot}::chassis::fpv_sight"
    # 1 is no lag: the view is bolted to the chassis.
    message.follow_pgain = 1.0
    message.track_pgain = 1.0
    return message


def release_track():
    """CameraTrack that lets go of the robot.

    CameraTracking keeps following on track_mode NONE, and only lets go on
    Escape in the 3D view, or when it cannot find its target. So it is
    given one that does not exist, and logs an "Unable to follow target"
    error for it.
    """
    message = CameraTrack()
    message.track_mode = CameraTrack.FOLLOW_LOOK_AT
    message.follow_target.name = NO_TARGET
    message.track_target.name = NO_TARGET
    return message


def view_camera(view):
    x, y, z, roll, pitch, yaw = VIEWS[view]
    message = GUICamera()
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.position.z = z

    # Roll, pitch, yaw to a quaternion.
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = message.pose.orientation
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return message


def key_action(key):
    """What a key asks for: ("view", name), ("fpv", None) to toggle FPV,
    ("robot", "robot_N"), ("escape", None), or None for any other key."""
    if key in VIEW_KEYS:
        return "view", VIEW_KEYS[key]
    if key == KEY_FPV:
        return FPV, None
    if KEY_F1 <= key < KEY_F1 + 9:
        return "robot", f"robot_{key - KEY_F1 + 1}"
    if key == KEY_ESCAPE:
        return "escape", None
    return None


class FpvToggle:

    def __init__(self, robot_ids, world="robogame_arena", fpv=True):
        self.robots = [f"robot_{robot_id}" for robot_id in robot_ids]
        self.world = world
        self.has_fpv = fpv
        self.robot = self.robots[0] if self.robots else ""
        self.fpv = False
        # The view over the arena, which 4 goes back to.
        self.view = START_VIEW
        self.lock = threading.Lock()
        # Requests to the GUI and server. A blocking request in
        # gz-transport's callback would hold up the next keypress, and one
        # from a thread started there fails, so a thread started here makes
        # them.
        self.requests = queue.Queue()
        threading.Thread(target=self.make_requests, daemon=True).start()

        self.node = Node()
        self.track = self.node.advertise("/gui/track", CameraTrack)
        self.state = self.node.advertise(STATE_TOPIC, StringMsg_V)
        self.node.subscribe(Int32, "/keyboard/keypress", self.on_key)
        self.node.subscribe(StringMsg, VIEW_TOPIC, self.on_view)
        threading.Thread(target=self.keep_publishing_state,
                         daemon=True).start()

        fpv_keys = (", 4 toggles FPV, F1 to F9 pick the robot"
                    if fpv else "")
        print(f"[fpv_toggle] 1 2 3 in the Gazebo window look down, over the "
              f"front and over the front right{fpv_keys}", flush=True)

    def on_key(self, message):
        action = key_action(message.data)
        if action is None:
            return

        kind, value = action
        with self.lock:
            if kind == "view":
                self.show_view(value)
            elif kind == FPV:
                if self.fpv:
                    self.show_view(self.view)
                else:
                    self.show_fpv(self.robot)
            elif kind == "robot":
                self.show_fpv(value)
            elif kind == "escape":
                # CameraTracking has let go itself, so the next 4 is FPV.
                self.fpv = False
                self.publish_state()

    def on_view(self, message):
        """A view asked for by the Camera view card, "VIEW ROBOT"."""
        view, _, robot = message.data.strip().partition(" ")
        robot = robot.strip()

        with self.lock:
            if robot in self.robots:
                self.robot = robot
            if view == FPV:
                self.show_fpv(robot or self.robot)
            elif view in VIEWS:
                self.show_view(view)
            else:
                print(f"[fpv_toggle] no view {view!r}", flush=True)
            self.publish_state()

    def make_requests(self):
        while True:
            self.requests.get()()

    def keep_publishing_state(self):
        # For a card that comes up after the last change.
        while True:
            with self.lock:
                self.publish_state()
            time.sleep(1.0)

    def publish_state(self):
        message = StringMsg_V()
        message.data.extend([FPV if self.fpv else self.view, self.robot])
        message.data.extend(self.robots if self.has_fpv else [])
        self.state.publish(message)

    def show_if_spawned(self, robot):
        ok, scene = self.node.request(f"/world/{self.world}/scene/info",
                                      Empty(), Empty, Scene, 2000)
        if not ok or robot not in {model.name for model in scene.model}:
            print(f"[fpv_toggle] no {robot} in the arena", flush=True)
            return

        with self.lock:
            if robot not in self.robots:
                self.robots.append(robot)
            self.show_fpv(robot)

    def show_fpv(self, robot):
        if not self.has_fpv:
            print("[fpv_toggle] no FPV cameras, fpv:=false", flush=True)
            return

        if robot not in self.robots:
            # Perhaps spawned since, for a controller.
            self.requests.put(lambda: self.show_if_spawned(robot))
            return

        self.robot = robot
        self.fpv = True
        self.track.publish(fpv_track(robot))
        self.publish_state()
        print(f"[fpv_toggle] FPV {robot}", flush=True)

    def show_view(self, view):
        was_fpv = self.fpv
        self.view = view
        self.fpv = False

        if was_fpv:
            self.track.publish(release_track())
        self.requests.put(lambda: self.fly_to(view, was_fpv))
        self.publish_state()
        print(f"[fpv_toggle] {view} view", flush=True)

    def fly_to(self, view, was_fpv):
        if was_fpv:
            # The GUI drops the follow on its next frame. Moving before
            # then loses to the follow.
            time.sleep(0.2)
        self.node.request("/gui/move_to/pose", view_camera(view),
                          GUICamera, Boolean, 1000)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robots", default="1",
                        help="comma separated robot numbers, the first is "
                             "the one 4 shows")
    parser.add_argument("--world", default="robogame_arena")
    parser.add_argument("--no-fpv", action="store_true",
                        help="the robots have no FPV camera")
    # ros2 run and launch_ros add their own arguments after ours.
    args, _ = parser.parse_known_args()
    robot_ids = [int(part) for part in args.robots.split(",")
                 if part.strip()]

    FpvToggle(robot_ids, args.world, fpv=not args.no_fpv)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()


if __name__ == "__main__":
    main()
