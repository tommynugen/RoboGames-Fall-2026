"""Switches the Gazebo window between the arena and a robot's FPV camera.

    ros2 run robogame_sim fpv_toggle --robots 1,2

With the 3D view focused, F toggles FPV, and a robot's number key looks
through that robot's camera instead, robots spawned since too. Escape
also leaves FPV, but stays where it is rather than going back over the
arena. The view sits where robot_sdf mounts the camera, on the front of the chassis, and turns with the chassis only,
so the arm moving does not move it.

Talks to the Gazebo GUI over gz-transport, not ROS: it reads the keys the
KeyPublisher GUI plugin publishes on /keyboard/keypress, and moves the GUI
camera with the CameraTracking plugin's /gui/track and /gui/move_to/pose.
sim.launch.py starts it, and adds KeyPublisher, unless fpv:=false.
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
from gz.transport import Node

from robogame_sim.robot_sdf import FPV_CAMERA

# Qt key codes, as KeyPublisher sends them. Upper and lower case are one.
KEY_F = 0x46
KEY_0 = 0x30
KEY_ESCAPE = 0x01000000

NO_TARGET = "fpv_off"

# The world's own view, over the arena's -y edge looking down at the
# middle: x y z roll pitch yaw.
OVERVIEW = (0.0, -1.1, 1.0, 0.0, 0.75, math.pi / 2)


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


def overview_camera():
    x, y, z, roll, pitch, yaw = OVERVIEW
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


class FpvToggle:

    def __init__(self, robot_ids, world="robogame_arena"):
        self.robots = [f"robot_{robot_id}" for robot_id in robot_ids]
        self.world = world
        self.robot = self.robots[0]
        self.fpv = False
        self.lock = threading.Lock()
        # Requests to the GUI and server. A blocking request in
        # gz-transport's callback would hold up the next keypress, and one
        # from a thread started there fails, so a thread started here makes
        # them.
        self.requests = queue.Queue()
        threading.Thread(target=self.make_requests, daemon=True).start()

        self.node = Node()
        self.track = self.node.advertise("/gui/track", CameraTrack)
        self.node.subscribe(Int32, "/keyboard/keypress", self.on_key)

        print(f"[fpv_toggle] F in the Gazebo window toggles FPV, "
              f"robots {', '.join(self.robots)}", flush=True)

    def on_key(self, message):
        key = message.data

        with self.lock:
            if key == KEY_F:
                self.show(self.robot, not self.fpv)
            elif key == KEY_ESCAPE:
                # CameraTracking has let go itself, so the next F is FPV.
                self.fpv = False
            elif KEY_0 <= key <= KEY_0 + 9:
                robot = f"robot_{key - KEY_0}"
                if robot in self.robots:
                    self.show(robot, True)
                else:
                    # Perhaps spawned since, for a controller.
                    self.requests.put(lambda: self.show_if_spawned(robot))

    def make_requests(self):
        while True:
            self.requests.get()()

    def show_if_spawned(self, robot):
        ok, scene = self.node.request(f"/world/{self.world}/scene/info",
                                      Empty(), Empty, Scene, 2000)
        if not ok or robot not in {model.name for model in scene.model}:
            print(f"[fpv_toggle] no {robot} in the arena", flush=True)
            return

        with self.lock:
            self.robots.append(robot)
            self.show(robot, True)

    def show(self, robot, fpv):
        self.robot = robot
        self.fpv = fpv

        if fpv:
            self.track.publish(fpv_track(robot))
            print(f"[fpv_toggle] FPV {robot}", flush=True)
            return

        self.track.publish(release_track())
        self.requests.put(self.fly_to_overview)
        print("[fpv_toggle] arena view", flush=True)

    def fly_to_overview(self):
        # The GUI drops the follow on its next frame. Moving before then
        # loses to the follow.
        time.sleep(0.2)
        self.node.request("/gui/move_to/pose", overview_camera(),
                          GUICamera, Boolean, 1000)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robots", default="1",
                        help="comma separated robot numbers, the first is "
                             "the one F shows")
    parser.add_argument("--world", default="robogame_arena")
    # ros2 run and launch_ros add their own arguments after ours.
    args, _ = parser.parse_known_args()
    robot_ids = [int(part) for part in args.robots.split(",")
                 if part.strip()]

    FpvToggle(robot_ids, args.world)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()


if __name__ == "__main__":
    main()
