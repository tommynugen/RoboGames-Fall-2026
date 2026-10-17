"""Gives every controller connected to this computer a robot in the sim.

    ros2 launch robogame_sim sim.launch.py robots:=1,2 controller:=hotplug \\
        drive:=2

The first controller found drives first_robot, which the launch spawned.
Each one connected after it gets a new robot, numbered on from the highest
already there: it is spawned on open ground with its own sim_firmware,
bridge and racing_teleop, as sim.launch.py spawns one. A controller that
drops and comes back drives the same robot again, found by its Bluetooth
address, and its robot waits where it stopped.

Virtual pads, like the X-Box pad Sunshine makes for a Moonlight stream or
Steam Input's copy of a controller, only drive while no real controller
has: they are often a second view of one already connected. A real
controller connecting later takes over the robot a virtual pad drove.

For the sim only. On the real arena robot_control.launch.py binds each
controller to the robot controllers.yaml gives it, and this would bind them
in the order they connect instead.
"""

import ctypes
import math
import os
import signal
import subprocess

import rclpy
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

from robogame_sim import arena_layout
from robogame_sim.gamepads import Gamepad, find_gamepads
from robogame_sim.robot_sdf import robot_sdf
from robogame_sim.spawning import (
    SPAWN_RADIUS,
    bridge_topics,
    spawn_spot,
)
from robogame_sim.terrain import Terrain

PR_SET_PDEATHSIG = 1


def die_with_parent():
    # So a robot's processes stop with this node even if it is killed.
    ctypes.CDLL("libc.so.6", use_errno=True).prctl(
        PR_SET_PDEATHSIG, signal.SIGTERM)


def executable(package, name):
    return os.path.join(get_package_prefix(package), "lib", package, name)


class Binding:
    """A controller and the robot it drives."""

    def __init__(self, robot_id, address, virtual=False):
        self.robot_id = robot_id
        self.address = address
        self.virtual = virtual
        self.pad = None
        self.publisher = None
        self.teleop = None
        self.last_publish = None


class ControllerRobots(Node):

    def __init__(self):
        super().__init__("controller_robots")

        self.declare_parameter("first_robot", 1)
        self.declare_parameter("robots", [1])
        self.declare_parameter("layout", "")
        self.declare_parameter("world", "robogame_arena")
        self.declare_parameter("drive_model", "ideal")
        self.declare_parameter("fpv", False)
        self.declare_parameter("deadzone", 0.05)
        self.declare_parameter("autorepeat_rate", 20.0)
        self.declare_parameter("scan_period", 1.0)

        def param(name):
            return self.get_parameter(name).value

        self.first_robot = param("first_robot")
        self.world = param("world")
        self.drive_model = param("drive_model")
        self.fpv = param("fpv")
        self.deadzone = param("deadzone")
        self.repeat = 1.0 / param("autorepeat_rate")

        self.ground = self.load_ground(param("layout"))
        self.bindings = {}
        self.standing_aside = set()
        self.processes = []
        self.poses = {}
        self.robots = set()

        for robot_id in param("robots"):
            self.track(robot_id)

        self.create_timer(param("scan_period"), self.scan)
        self.create_timer(0.02, self.poll)

        self.get_logger().info(
            f"The first controller drives robot {self.first_robot}, and "
            f"each one connected after it gets a robot of its own")
        self.scan()

    def load_ground(self, layout):
        if layout:
            return arena_layout.terrain(arena_layout.load(layout))

        return Terrain.from_sdf(os.path.join(
            get_package_share_directory("robogame_sim"), "models",
            arena_layout.CAD_MODEL, "model.sdf"))

    def track(self, robot_id):
        """Follows where a robot is, so new ones spawn away from it."""
        self.robots.add(robot_id)

        def remember(message, robot_id=robot_id):
            p = message.pose.pose.position
            self.poses[robot_id] = (p.x, p.y)

        self.create_subscription(
            Odometry, f"/model/robot_{robot_id}/odometry", remember, 10)

    # -- controllers -----------------------------------------------------

    def scan(self):
        pads = find_gamepads()
        real = (any(not virtual for *_, virtual in pads)
                or any(not b.virtual for b in self.bindings.values()))

        for device, name, address, virtual in pads:
            binding = self.bindings.get(address)

            if binding is not None and binding.pad is not None:
                continue

            if binding is None:
                if virtual and real:
                    if address not in self.standing_aside:
                        self.standing_aside.add(address)
                        self.get_logger().info(
                            f"Leaving out {name} {address} on {device}, a "
                            f"virtual pad, while a real controller drives")
                    continue

                binding = self.bind(address, virtual, name)

            try:
                binding.pad = Gamepad(device, self.deadzone)
            except OSError as error:
                self.get_logger().warning(
                    f"Cannot open {device} for {name} {address}: {error}")
                continue

            binding.teleop = self.start(
                "racing_teleop", binding.robot_id,
                executable("robogame_sim", "racing_teleop"),
                "--ros-args", "-r", f"__ns:=/robot_{binding.robot_id}",
                "-p", f"robot_id:={binding.robot_id}",
                "-p", f"controller_address:={address}")
            self.get_logger().info(
                f"{name} {address} on {device} drives robot "
                f"{binding.robot_id}")

    def bind(self, address, virtual, name):
        stand_in = next(
            (b for b in self.bindings.values() if b.virtual), None)

        if stand_in is not None and not virtual:
            # Its robot goes to the real controller, rather than a new one.
            del self.bindings[stand_in.address]
            if stand_in.pad is not None:
                stand_in.pad.close()
            self.stop(stand_in.teleop)
            binding = Binding(stand_in.robot_id, address)
            binding.publisher = stand_in.publisher
            self.bindings[address] = binding
            self.get_logger().info(
                f"{name} {address} takes robot {binding.robot_id} over from "
                f"the virtual pad {stand_in.address}")
            return binding

        robot_id = (self.first_robot if not self.bindings
                    else max(self.robots) + 1)
        binding = Binding(robot_id, address, virtual)
        self.bindings[address] = binding
        binding.publisher = self.create_publisher(
            Joy, f"/robot_{robot_id}/joy", 10)

        if robot_id not in self.robots:
            self.spawn(robot_id)

        return binding

    def poll(self):
        now = self.get_clock().now()

        for binding in self.bindings.values():
            if binding.pad is None:
                continue

            changed = binding.pad.poll()

            if changed is None:
                self.drop(binding)
                continue

            due = (binding.last_publish is None
                   or (now - binding.last_publish).nanoseconds * 1e-9
                   >= self.repeat)

            if changed or due:
                message = Joy()
                message.header.stamp = now.to_msg()
                message.header.frame_id = "joy"
                message.axes = list(binding.pad.axes)
                message.buttons = list(binding.pad.buttons)
                binding.publisher.publish(message)
                binding.last_publish = now

    def drop(self, binding):
        binding.pad.close()
        binding.pad = None
        # racing_teleop lets go of the motion sensor when it goes, so a new
        # one is started on reconnecting.
        self.stop(binding.teleop)
        binding.teleop = None
        self.get_logger().warning(
            f"Controller {binding.address} disconnected, robot "
            f"{binding.robot_id} waits for it to come back")

    # -- robots ----------------------------------------------------------

    def spawn(self, robot_id):
        robot = f"robot_{robot_id}"
        # On the spawn circle, each new robot a little over a third of a
        # turn on from the last, then moved clear of the robots there now.
        angle = -3 * math.pi / 4 + robot_id * 2.4
        x, y = SPAWN_RADIUS * math.cos(angle), SPAWN_RADIUS * math.sin(angle)
        taken = list(self.poses.values())
        x, y, z, yaw, _ = spawn_spot(self.ground, x, y, angle + math.pi,
                                     taken)
        self.track(robot_id)
        # Until its odometry comes, so the next one is not put on it.
        self.poses[robot_id] = (x, y)
        self.get_logger().info(
            f"Spawning {robot} at {x:.2f}, {y:.2f} for the new controller")

        self.start(
            "spawn", robot_id, executable("ros_gz_sim", "create"),
            "-world", self.world, "-name", robot,
            "-string", robot_sdf(robot_id, fpv=self.fpv),
            "-x", f"{x:.3f}", "-y", f"{y:.3f}", "-z", f"{z:.3f}",
            "-Y", f"{yaw:.3f}")
        self.start(
            "sim_firmware", robot_id,
            executable("robogame_sim", "sim_firmware"),
            "--ros-args", "-r", f"__ns:=/{robot}",
            "-p", f"robot_id:={robot_id}",
            "-p", f"drive_model:={self.drive_model}",
            "-p", "use_sim_time:=true")
        self.start(
            "bridge", robot_id,
            executable("ros_gz_bridge", "parameter_bridge"),
            *bridge_topics(robot),
            "--ros-args", "-r", f"__node:={robot}_bridge",
            "-r", f"/model/{robot}/tf:=/tf")

    def start(self, what, robot_id, *command):
        process = subprocess.Popen(command, preexec_fn=die_with_parent)
        self.processes.append(process)
        self.get_logger().debug(
            f"Started {what} for robot {robot_id}, pid {process.pid}")
        return process

    def stop(self, process, timeout=3.0):
        if process is None or process.poll() is not None:
            return

        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout)
        except subprocess.TimeoutExpired:
            process.kill()

    def shutdown(self):
        for binding in self.bindings.values():
            if binding.pad is not None:
                binding.pad.close()

        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)

        for process in self.processes:
            self.stop(process)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerRobots()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # launch passes Ctrl-C on as a second SIGINT, which would otherwise
        # cut stopping the robots' processes short.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
