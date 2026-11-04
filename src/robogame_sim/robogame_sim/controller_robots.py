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

The robot console (robot_console.py) moves a controller to another robot,
or takes it off every robot, with a JSON message on
/controller_robots/assign: {"address": "AA:BB:...", "robot": 2}, or
"robot": null. A controller moved onto a robot another one drives swaps
robots with it. Every controller and the robot it drives are published,
latched, on /controller_robots/state, with the robots it knows of.

The console also adds and removes robots, on /controller_robots/robots:
{"add": true} spawns a new one, numbered on from the highest there has
been, and {"remove": 2} takes robot 2 out of the world, its controller
left on no robot. With spawn_for_controllers false, as sim.launch.py runs
it when no controller is asked for, a controller that connects drives no
robot until it is given one in the console.

On the real arena, run it with real:=true, or press Start under
Controllers in the robot console, which does that:

    ros2 run robogame_sim controller_robots --ros-args -p real:=true

It then spawns nothing and adds no robots. It finds each real robot by its
/robot_N/cmd_vel, which the robot's bridge listens on, and controllers are
given robots in the console, as in the sim, in place of controllers.yaml.

Each robot has its controls: "classic", arena_perception's
controller_input, as robot_control.launch.py runs it, or "racing",
racing_teleop. Real robots start on classic and sim robots on racing, so
a robot drives as it did until its controls are changed in the console,
with {"robot": 2, "controls": "racing"} on /controller_robots/robots.
"""

import ctypes
import json
import math
import os
import re
import signal
import subprocess

import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import String

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

# A real robot's bridge listens on its cmd_vel.
ROBOT_TOPIC = re.compile(r"^/robot_(\d+)/cmd_vel$")
# Each robot's controls, by name: the package and program that turn its
# /robot_N/joy into /robot_N/cmd_vel.
CONTROLS = {
    "classic": ("arena_perception", "controller_input"),
    "racing": ("robogame_sim", "racing_teleop"),
}

# Kept for anyone subscribing later, as the robot console does.
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def die_with_parent():
    # So a robot's processes stop with this node even if it is killed.
    ctypes.CDLL("libc.so.6", use_errno=True).prctl(
        PR_SET_PDEATHSIG, signal.SIGTERM)


def executable(package, name):
    return os.path.join(get_package_prefix(package), "lib", package, name)


def installed(package):
    try:
        get_package_prefix(package)
        return True
    except PackageNotFoundError:
        return False


def teleop_command(controls, robot_id, address):
    """The command that runs a robot's controls for a controller."""
    package, name = CONTROLS[controls]
    command = [executable(package, name),
               "--ros-args", "-r", f"__ns:=/robot_{robot_id}",
               "-p", f"robot_id:={robot_id}"]
    if controls == "racing":
        command += ["-p", f"controller_address:={address}"]
    return command


def robots_on(topics):
    """Robot numbers with a cmd_vel among (topic, listeners) pairs."""
    return {int(m.group(1)) for topic, listeners in topics
            if listeners and (m := ROBOT_TOPIC.match(topic))}


class Binding:
    """A controller and the robot it drives, None when the robot console
    has taken it off every robot."""

    def __init__(self, robot_id, address, virtual=False, name=""):
        self.robot_id = robot_id
        self.address = address
        self.virtual = virtual
        self.name = name
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
        self.declare_parameter("spawn_for_controllers", True)
        # The real arena: spawn nothing, find robots by their cmd_vel.
        self.declare_parameter("real", False)
        # Controls a robot starts on, classic on the real arena and racing
        # in the sim when empty.
        self.declare_parameter("controls", "")

        def param(name):
            return self.get_parameter(name).value

        self.first_robot = param("first_robot")
        self.world = param("world")
        self.drive_model = param("drive_model")
        self.fpv = param("fpv")
        self.deadzone = param("deadzone")
        self.repeat = 1.0 / param("autorepeat_rate")
        self.real = param("real")
        self.spawn_for_controllers = (param("spawn_for_controllers")
                                      and not self.real)
        self.options = [c for c in CONTROLS
                        if installed(CONTROLS[c][0])]
        self.default_controls = param("controls") or (
            "classic" if self.real else "racing")
        if self.default_controls not in self.options:
            self.get_logger().warning(
                f"No {self.default_controls} controls here, using racing")
            self.default_controls = "racing"
        self.controls = {}

        self.ground = None if self.real else self.load_ground(param("layout"))
        self.bindings = {}
        self.standing_aside = set()
        self.processes = []
        self.poses = {}
        self.robots = set()
        # Robot numbers used so far, removed ones too: a robot the launch
        # started still has its bridge, so its number is not given again.
        self.used = set()
        self.robot_processes = {}
        self.odometry = {}

        self.state_publisher = self.create_publisher(
            String, "/controller_robots/state", LATCHED)
        self.create_subscription(
            String, "/controller_robots/assign", self.on_assign, 10)
        self.create_subscription(
            String, "/controller_robots/robots", self.on_robots, 10)

        # Real robots are found as they come, by find_real_robots.
        for robot_id in [] if self.real else param("robots"):
            self.track(robot_id)

        self.create_timer(param("scan_period"), self.scan)
        self.create_timer(0.02, self.poll)

        self.get_logger().info(
            f"The first controller drives robot {self.first_robot}, and "
            f"each one connected after it gets a robot of its own"
            if self.spawn_for_controllers else
            "Controllers drive no robot until given one in the console"
            + (", on the real arena" if self.real else ""))
        self.publish_state()
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
        self.used.add(robot_id)
        if self.real:
            return

        def remember(message, robot_id=robot_id):
            p = message.pose.pose.position
            self.poses[robot_id] = (p.x, p.y)

        self.odometry[robot_id] = self.create_subscription(
            Odometry, f"/model/robot_{robot_id}/odometry", remember, 10)

    # -- controllers -----------------------------------------------------

    def scan(self):
        if self.real:
            self.find_real_robots()

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

            self.drive(binding)
            self.get_logger().info(
                f"{name} {address} on {device} drives "
                + (f"robot {binding.robot_id}"
                   if binding.robot_id is not None else "no robot"))
            self.publish_state()

    def find_real_robots(self):
        topics = [(topic, self.count_subscribers(topic))
                  for topic, _ in self.get_topic_names_and_types()]
        found = robots_on(topics) - self.robots
        for robot_id in sorted(found):
            self.track(robot_id)
            self.get_logger().info(f"Found real robot {robot_id}")
        if found:
            self.publish_state()

    def controls_of(self, robot_id):
        return self.controls.get(robot_id, self.default_controls)

    def drive(self, binding):
        """Starts a connected controller's robot's controls."""
        if binding.pad is None or binding.robot_id is None:
            return

        controls = self.controls_of(binding.robot_id)
        binding.teleop = self.start(
            controls, binding.robot_id,
            *teleop_command(controls, binding.robot_id, binding.address))

    def bind(self, address, virtual, name):
        stand_in = next(
            (b for b in self.bindings.values()
             if b.virtual and b.robot_id is not None), None)

        if stand_in is not None and not virtual:
            # Its robot goes to the real controller, rather than a new one.
            del self.bindings[stand_in.address]
            if stand_in.pad is not None:
                stand_in.pad.close()
            self.stop(stand_in.teleop)
            binding = Binding(stand_in.robot_id, address, name=name)
            binding.publisher = stand_in.publisher
            self.bindings[address] = binding
            self.get_logger().info(
                f"{name} {address} takes robot {binding.robot_id} over from "
                f"the virtual pad {stand_in.address}")
            return binding

        if not self.spawn_for_controllers:
            binding = Binding(None, address, virtual, name)
            self.bindings[address] = binding
            self.publish_state()
            return binding

        robot_id = (self.first_robot if not self.bindings
                    else self.next_robot())
        binding = Binding(robot_id, address, virtual, name)
        self.bindings[address] = binding
        binding.publisher = self.joy_publisher(robot_id)

        if robot_id not in self.robots:
            self.spawn(robot_id)

        self.publish_state()
        return binding

    def poll(self):
        now = self.get_clock().now()

        for binding in self.bindings.values():
            if binding.pad is None:
                continue

            if binding.publisher is None:
                # On no robot: read on, so a dropped controller is noticed.
                if binding.pad.poll() is None:
                    self.drop(binding)
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
        self.publish_state()

    # -- the robot console -----------------------------------------------

    def joy_publisher(self, robot_id):
        if robot_id is None:
            return None
        return self.create_publisher(Joy, f"/robot_{robot_id}/joy", 10)

    def on_assign(self, message):
        try:
            asked = json.loads(message.data)
            address = asked["address"]
            robot_id = asked.get("robot")
            robot_id = None if robot_id is None else int(robot_id)
        except (ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(
                f"Bad assignment {message.data!r}: {error}")
            return

        binding = self.bindings.get(address)
        if binding is None:
            self.get_logger().warning(f"No controller {address} to move")
            return
        if robot_id is not None and robot_id not in self.robots:
            self.get_logger().warning(
                f"No robot {robot_id} to give {address}")
            return
        if robot_id == binding.robot_id:
            self.publish_state()
            return

        # The controller on that robot already, if any, gets this one's.
        other = next((b for b in self.bindings.values()
                      if b is not binding and robot_id is not None
                      and b.robot_id == robot_id), None)
        moves = [(binding, robot_id)]
        if other is not None:
            moves.append((other, binding.robot_id))

        for moved, _ in moves:
            self.stop(moved.teleop)
            moved.teleop = None
            if moved.publisher is not None:
                self.destroy_publisher(moved.publisher)

        for moved, new_robot in moves:
            moved.robot_id = new_robot
            moved.publisher = self.joy_publisher(new_robot)
            moved.last_publish = None
            self.drive(moved)
            self.get_logger().info(
                f"Controller {moved.address} now drives "
                + (f"robot {new_robot}" if new_robot is not None
                   else "no robot"))

        self.publish_state()

    def on_robots(self, message):
        try:
            asked = json.loads(message.data)
            remove = asked.get("remove")
            remove = None if remove is None else int(remove)
        except (ValueError, TypeError, AttributeError) as error:
            self.get_logger().warning(
                f"Bad robots message {message.data!r}: {error}")
            return

        if "controls" in asked:
            self.set_controls(asked.get("robot"), asked["controls"])
        elif self.real and (asked.get("add") or remove is not None):
            self.get_logger().warning(
                "Real robots cannot be added or removed here")
        else:
            if asked.get("add"):
                self.spawn(self.next_robot())
            if remove is not None:
                self.remove(remove)
        self.publish_state()

    def set_controls(self, robot_id, controls):
        try:
            robot_id = int(robot_id)
        except (TypeError, ValueError):
            robot_id = None
        if robot_id not in self.robots or controls not in self.options:
            self.get_logger().warning(
                f"Cannot give robot {robot_id} {controls} controls")
            return
        if controls == self.controls_of(robot_id):
            return

        self.controls[robot_id] = controls
        # Its controller's old controls stop before the new ones start, so
        # the two never both drive it.
        for binding in self.bindings.values():
            if binding.robot_id == robot_id:
                self.stop(binding.teleop)
                binding.teleop = None
                self.drive(binding)
        self.get_logger().info(f"Robot {robot_id} has {controls} controls")

    def next_robot(self):
        return max(self.used | {self.first_robot - 1}) + 1

    def remove(self, robot_id):
        if robot_id not in self.robots:
            self.get_logger().warning(f"No robot {robot_id} to remove")
            return

        for binding in self.bindings.values():
            if binding.robot_id == robot_id:
                self.stop(binding.teleop)
                binding.teleop = None
                if binding.publisher is not None:
                    self.destroy_publisher(binding.publisher)
                binding.publisher = None
                binding.robot_id = None

        robot = f"robot_{robot_id}"
        try:
            subprocess.run(
                ["gz", "service", "-s", f"/world/{self.world}/remove",
                 "--reqtype", "gz.msgs.Entity",
                 "--reptype", "gz.msgs.Boolean", "--timeout", "2000",
                 "--req", f'name: "{robot}" type: MODEL'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=5, check=False)
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().warning(f"Cannot remove {robot}: {error}")
        # Its firmware and bridge, if this node started them. A robot the
        # launch started keeps its, idle, until the launch stops.
        for process in self.robot_processes.pop(robot_id, []):
            self.stop(process)
        self.destroy_subscription(self.odometry.pop(robot_id))
        self.controls.pop(robot_id, None)
        self.robots.discard(robot_id)
        self.poses.pop(robot_id, None)
        self.get_logger().info(f"Removed {robot}")

    def publish_state(self):
        controllers = [
            {"address": b.address, "name": b.name, "robot": b.robot_id,
             "connected": b.pad is not None, "virtual": b.virtual}
            for b in self.bindings.values()]
        self.state_publisher.publish(String(data=json.dumps({
            "controllers": controllers,
            "robots": sorted(self.robots),
            "real": self.real,
            "controls": {str(r): self.controls_of(r) for r in self.robots},
            "options": self.options,
        })))

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
        self.get_logger().info(f"Spawning {robot} at {x:.2f}, {y:.2f}")

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
        if what in ("sim_firmware", "bridge"):
            self.robot_processes.setdefault(robot_id, []).append(process)
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
