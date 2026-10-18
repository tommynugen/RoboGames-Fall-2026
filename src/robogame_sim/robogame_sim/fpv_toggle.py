"""Moves the Gazebo window between views over the arena and a robot's FPV
camera.

    ros2 run robogame_sim fpv_toggle --robots 1,2

With the 3D view focused, 1 looks straight down on the arena, 2 over its
front (-y) edge, 3 over its front right corner, and 4 through the driven
robot's FPV camera. F1 to F9 look through that robot's camera instead,
robots spawned since too. A key switches to its view; pressed again, it
does nothing. Escape leaves FPV but stays where it is, rather than going
over the arena. The FPV view
sits where robot_sdf mounts the camera, on the front of the chassis, and
turns with the chassis only, so the arm moving does not move it.

The Camera view card in the window's side panel (robogame_gui) does the
same from two drop downs. It asks for a view on /robogame/view with the
view's name and a robot, "top robot_2" or "fpv robot_1", and fills its
drop downs from /robogame/view/state, which this publishes on every
change and once a second: the view, the robot, then every robot. It
also sends the 3D view's width over its height on /robogame/view/aspect,
and the views fit the arena to it.

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
from gz.msgs.double_pb2 import Double
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

# Views over the arena, pitch and yaw: each is as close as the whole arena
# fits, with the arena in the middle of the picture. Straight down is a
# hair off 90 degrees, so the view still knows which way is up: the front
# edge at the bottom.
VIEWS = {
    "top": (math.pi / 2 - 0.001, math.pi / 2),
    "front": (0.75, math.pi / 2),
    "front_right": (0.72, 3 * math.pi / 4),
}
VIEW_KEYS = {KEY_1 + index: name for index, name in enumerate(VIEWS)}

# The arena's floor, 12 tiles of 4 inches.
ARENA_HALF = 6 * 0.1016
ARENA_CORNERS = [(x, y, 0.0)
                 for x in (-ARENA_HALF, ARENA_HALF)
                 for y in (-ARENA_HALF, ARENA_HALF)]
# The arena fills this much of the picture, across or up, whichever is
# tighter, leaving room for the walls along its edges.
FILL = 0.95
# The Gazebo window's 3D view: 90 degrees across, whatever its shape.
HFOV = math.pi / 2
# Width over height of the 3D view, until the Camera view card says: the
# window as Gazebo opens it, side panels and all.
DEFAULT_ASPECT = 577 / 797

# The view the window starts on, put in gui.config by the launch.
START_VIEW = "front"

VIEW_TOPIC = "/robogame/view"
STATE_TOPIC = "/robogame/view/state"
ASPECT_TOPIC = "/robogame/view/aspect"


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


def camera_axes(pitch, yaw):
    """The camera's forward, left and up, in the world, for pitch down and
    yaw, with no roll."""
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    forward = (cp * cy, cp * sy, -sp)
    left = (-sy, cy, 0.0)
    up = (sp * cy, sp * sy, cp)
    return forward, left, up


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


# A corner is dot(p, forward) - along in front of a camera along forward,
# and dot(p, axis) - shift off the middle of the picture along one of its
# axes. Across and up are worked out apart: shifting the camera along
# left or up does not change how far ahead a corner is.


def reach(ahead, off, tan_half):
    """How far forward the camera can come with every corner inside
    tan_half of the middle along one axis, shifted as best suits.

    A corner fits for along up to ahead - abs(off - shift) / tan_half. The
    nearest of those is highest at one shift, found by thirds."""
    def nearest(shift):
        return min(f - abs(o - shift) / tan_half
                   for f, o in zip(ahead, off))

    low, high = min(off), max(off)
    for _ in range(60):
        third = (high - low) / 3
        if nearest(low + third) < nearest(high - third):
            low += third
        else:
            high -= third
    return nearest((low + high) / 2)


def centre(ahead, off, along):
    """The shift along one axis that puts the corners as far either side of
    the middle of the picture, for a camera along forward."""
    def lean(shift):
        seen = [(o - shift) / (f - along) for f, o in zip(ahead, off)]
        return max(seen) + min(seen)

    # lean falls as shift grows.
    low, high = min(off), max(off)
    for _ in range(60):
        middle = (low + high) / 2
        if lean(middle) > 0:
            low = middle
        else:
            high = middle
    return (low + high) / 2


def view_pose(view, aspect=DEFAULT_ASPECT):
    """x y z roll pitch yaw for view, as close as the arena's floor still
    fits a 3D view aspect wide for each 1 high, in the middle of it."""
    pitch, yaw = VIEWS[view]
    forward, left, up = camera_axes(pitch, yaw)
    across = math.tan(HFOV / 2) * FILL
    upward = across / aspect

    ahead = [dot(p, forward) for p in ARENA_CORNERS]
    sideways = [dot(p, left) for p in ARENA_CORNERS]
    upwards = [dot(p, up) for p in ARENA_CORNERS]
    along = min(reach(ahead, sideways, across),
                reach(ahead, upwards, upward))
    side = centre(ahead, sideways, along)
    rise = centre(ahead, upwards, along)

    x, y, z = (along * f + side * s + rise * u
               for f, s, u in zip(forward, left, up))
    return x, y, z, 0.0, pitch, yaw


def view_camera(view, aspect=DEFAULT_ASPECT):
    x, y, z, roll, pitch, yaw = view_pose(view, aspect)
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
        # The view shown: one of VIEWS, FPV, or "" once Escape or the
        # mouse has taken the camera somewhere else.
        self.current = START_VIEW
        self.aspect = DEFAULT_ASPECT
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
        self.node.subscribe(Double, ASPECT_TOPIC, self.on_aspect)
        threading.Thread(target=self.keep_publishing_state,
                         daemon=True).start()

        fpv_keys = (", 4 FPV, F1 to F9 pick the robot"
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
                self.show_fpv(self.robot)
            elif kind == "robot":
                self.show_fpv(value)
            elif kind == "escape":
                # CameraTracking has let go itself, so the next 4 is FPV
                # again, and any view key moves the camera.
                self.current = ""
                self.publish_state()

    def on_view(self, message):
        """A view asked for by the Camera view card, "VIEW ROBOT"."""
        view, _, robot = message.data.strip().partition(" ")
        robot = robot.strip()

        with self.lock:
            if view == FPV:
                self.show_fpv(robot or self.robot)
            elif view in VIEWS:
                # The robot 4 looks through next.
                if robot in self.robots:
                    self.robot = robot
                self.show_view(view)
            else:
                print(f"[fpv_toggle] no view {view!r}", flush=True)
            self.publish_state()

    def on_aspect(self, message):
        """The 3D view's shape, from the Camera view card: the view shown
        is fitted to it again when it changes, as when the window is
        resized."""
        aspect = message.data
        with self.lock:
            if aspect <= 0 or abs(aspect / self.aspect - 1) < 0.02:
                return
            self.aspect = aspect
            if self.current in VIEWS:
                view = self.current
                self.requests.put(lambda: self.fly_to(view, False))

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
        message.data.extend([self.current, self.robot])
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

        if self.current == FPV and robot == self.robot:
            return

        self.robot = robot
        self.current = FPV
        self.track.publish(fpv_track(robot))
        self.publish_state()
        print(f"[fpv_toggle] FPV {robot}", flush=True)

    def show_view(self, view):
        if view == self.current:
            return

        was_fpv = self.current == FPV
        self.current = view

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
        self.node.request("/gui/move_to/pose",
                          view_camera(view, self.aspect),
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
