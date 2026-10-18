"""Gazebo arena with one simulated robot per robot number.

    ros2 launch robogame_sim sim.launch.py
    ros2 launch robogame_sim sim.launch.py robots:=1,2,3
    ros2 launch robogame_sim sim.launch.py controller:=joy
    ros2 launch robogame_sim sim.launch.py controller:=classic
    ros2 launch robogame_sim sim.launch.py autonomy:=true
    ros2 launch robogame_sim sim.launch.py robots:=1,2 chase:=1,2
    ros2 launch robogame_sim sim.launch.py drive_model:=firmware gui:=false
    ros2 launch robogame_sim sim.launch.py layout:=my_setup
    ros2 launch robogame_sim sim.launch.py partition:=second
    ros2 launch robogame_sim sim.launch.py fpv:=false controller:=joy
    ros2 launch robogame_sim sim.launch.py robots:=1,2 chase:=1,2 \
        controller:=hotplug drive:=2

layout names an arena layout made with arena_editor (or a path to one), and
the world is built from it; cad uses the Onshape export as it is.

Only one sim can run in a Gazebo partition, since the world's name is the
same. If one is already running this stops with a message rather than
attaching to it; partition:=NAME runs a second one beside it.

Robots spawn on a circle facing the middle, each moved to the nearest open
floor when its spot is on or wedged against a module.

Each robot listens on /robot_N/cmd_vel and /robot_N/arm_command, the same
topics microcontroller_node takes, so do not run the BLE bridge alongside.

The controller drives robot drive, the first in robots unless set.
controller:=hotplug gives it the first controller connected, and every
controller connected after that a new robot of its own (controller_robots).
That is for the sim only: the real arena binds controllers to robots by
address. Virtual pads, such as Sunshine's for a Moonlight stream, are left
out while a real controller drives.

With the window up, every robot's chassis carries an FPV camera on its
front, unless fpv:=false. Click the Gazebo window's 3D view, then 1 looks
straight down on the arena, 2 over its front, 3 over its front right
corner, 4 switches to the driven robot's camera and back, and F1 to F9
look through that robot's instead. The Camera view card in the side panel
does the same, when robogame_gui is built. See fpv_toggle.py.
"""

import difflib
import glob
import os
import re
import subprocess
import tempfile

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_sim.actions import GzServer

from robogame_sim import arena_layout
from robogame_sim.fpv_toggle import OVERVIEW
from robogame_sim.robot_sdf import robot_sdf
from robogame_sim.spawning import bridge_topics, spawn_pose, spawn_spot
from robogame_sim.terrain import Terrain

WORLD_NAME = "robogame_arena"


def gui_command():
    # gz sim -g starts the GUI client through sh, which Ctrl-C does not
    # pass on to, so the window outlives the launch. Run the client itself.
    try:
        prefix = get_package_prefix("gz_sim_vendor")
    except PackageNotFoundError:
        return ["gz", "sim", "-g"]

    clients = sorted(glob.glob(os.path.join(
        prefix, "opt", "gz_sim_vendor", "libexec", "gz", "sim*",
        "gz-sim-gui-client")))

    return clients[-1:] or ["gz", "sim", "-g"]


# Publishes the keys pressed in the 3D view for fpv_toggle. Its card only
# says so, so it is shrunk out of the way like the other helpers.
KEY_PUBLISHER = """
<plugin filename="KeyPublisher" name="Key publisher">
  <gz-gui>
    <property key="resizable" type="bool">false</property>
    <property key="width" type="double">5</property>
    <property key="height" type="double">5</property>
    <property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property>
  </gz-gui>
</plugin>
"""

# Drop downs in the side panel for fpv_toggle's views, from robogame_gui.
VIEW_CARD = """
<plugin filename="RobogameView" name="Camera view">
  <gz-gui>
    <title>Camera view</title>
    <property key="state" type="string">docked</property>
  </gz-gui>
</plugin>
"""

# The FPV view sits 2 cm under the arm, nearer than the default near clip.
CAMERA_CLIP = "<camera_clip><near>0.005</near><far>50</far></camera_clip>"


def default_gui_config():
    # The one gz sim -g would load: the user's copy, else the installed one.
    found = sorted(glob.glob(os.path.expanduser("~/.gz/sim/*/gui.config")))

    try:
        prefix = get_package_prefix("gz_sim_vendor")
        found = found or sorted(glob.glob(os.path.join(
            prefix, "opt", "gz_sim_vendor", "share", "gz", "gz-sim*",
            "gui", "gui.config")))
    except PackageNotFoundError:
        pass

    return found[-1] if found else None


def view_card_path():
    """Where robogame_gui put the Camera view card, or None if it is not
    built."""
    try:
        prefix = get_package_prefix("robogame_gui")
    except PackageNotFoundError:
        return None

    path = os.path.join(prefix, "lib", "robogame_gui")
    return path if os.path.exists(
        os.path.join(path, "libRobogameView.so")) else None


def view_gui_config(card):
    """The default Gazebo GUI, starting over the arena and publishing keys
    for fpv_toggle, with the Camera view card if card, or None if the
    default cannot be found."""
    default = default_gui_config()
    if default is None:
        return None

    with open(default) as config:
        text = config.read()

    overview = " ".join(f"{value:.4f}" for value in OVERVIEW)
    text = re.sub(r"<camera_pose>[^<]*</camera_pose>",
                  f"<camera_pose>{overview}</camera_pose>{CAMERA_CLIP}",
                  text, count=1)
    text += KEY_PUBLISHER
    if card:
        text += VIEW_CARD

    path = os.path.join(
        tempfile.mkdtemp(prefix="robogame_sim_"), "gui.config")
    with open(path, "w") as config:
        config.write(text)

    return path


def package_exists(name):
    try:
        get_package_share_directory(name)
    except PackageNotFoundError:
        return False

    return True


def world_running(name, env):
    """Whether a Gazebo world of this name is already up in env's partition.
    False when gz cannot tell, so the launch goes ahead as before."""
    try:
        services = subprocess.run(
            ["gz", "service", "-l"], capture_output=True, text=True,
            timeout=5, env=env).stdout
    except (OSError, subprocess.TimeoutExpired):
        return False

    return f"/world/{name}/" in services


LAUNCH_ARGUMENTS = (
    "robots", "gui", "drive_model", "controller", "joy_device_id",
    "motion_device", "autonomy", "chase", "layout", "partition", "fpv",
    "drive", "speed", "step",
)


def set_physics(world, speed, step):
    """A copy of the world file running at real time factor speed, with
    physics steps of step seconds."""
    with open(world) as file:
        text = file.read()

    text = re.sub(r"<real_time_factor>[^<]*</real_time_factor>",
                  f"<real_time_factor>{speed}</real_time_factor>", text,
                  count=1)
    text = re.sub(r"<max_step_size>[^<]*</max_step_size>",
                  f"<max_step_size>{step}</max_step_size>", text, count=1)
    path = os.path.join(tempfile.mkdtemp(prefix="robogame_world_"),
                        os.path.basename(world))
    with open(path, "w") as file:
        file.write(text)

    return path


def check_argument_names(context):
    # ros2 launch takes any name:=value without complaint, so a typo such
    # as controler:=joy would otherwise quietly start no controller.
    for name in context.launch_configurations:
        if name in LAUNCH_ARGUMENTS:
            continue

        close = difflib.get_close_matches(name, LAUNCH_ARGUMENTS, n=1)
        hint = f", did you mean {close[0]}?" if close else ""
        raise RuntimeError(
            f"Unknown launch argument {name}{hint} Arguments are "
            + ", ".join(LAUNCH_ARGUMENTS))


def launch_setup(context):
    check_argument_names(context)

    robot_ids = [
        int(part)
        for part in LaunchConfiguration("robots").perform(context).split(",")
        if part.strip()
    ]
    gui = LaunchConfiguration("gui").perform(context) == "true"
    drive_model = LaunchConfiguration("drive_model").perform(context)
    controller = LaunchConfiguration("controller").perform(context)
    autonomy = LaunchConfiguration("autonomy").perform(context) == "true"
    chase = [
        int(part)
        for part in LaunchConfiguration("chase").perform(context).split(",")
        if part.strip()
    ]
    fpv = LaunchConfiguration("fpv").perform(context) == "true"
    drive = LaunchConfiguration("drive").perform(context).strip()
    driven = int(drive) if drive else robot_ids[0]

    if driven not in robot_ids:
        raise RuntimeError(
            f"drive:={driven} is not one of robots:={robot_ids}")

    # A second gzserver with the same world name refuses to start, and
    # everything else in this launch then attaches to the sim already
    # running, which looks like a frozen sim. Say so instead.
    partition = LaunchConfiguration("partition").perform(context)
    env = dict(os.environ)
    if partition:
        env["GZ_PARTITION"] = partition

    if world_running(WORLD_NAME, env):
        where = (f"partition {env['GZ_PARTITION']}"
                 if env.get("GZ_PARTITION") else "the default partition")
        raise RuntimeError(
            f"A Gazebo world named {WORLD_NAME} is already running in "
            f"{where}, probably another sim.launch.py. Stop it with Ctrl-C "
            f"in its terminal, or run this one beside it with "
            f"partition:=NAME")

    share = get_package_share_directory("robogame_sim")
    world = os.path.join(share, "worlds", f"{WORLD_NAME}.sdf")
    layout_name = LaunchConfiguration("layout").perform(context)

    if layout_name == "cad":
        layout_path = ""
        ground = Terrain.from_sdf(os.path.join(
            share, "models", arena_layout.CAD_MODEL, "model.sdf"))
    else:
        layout_path = arena_layout.find_layout(layout_name)
        layout = arena_layout.load(layout_path)
        ground = arena_layout.terrain(layout)
        world = arena_layout.write_world(layout, world, ground)

        for problem in arena_layout.check(layout):
            print(f"[sim.launch] layout {layout_name}: {problem['type']} at "
                  f"col {problem['col']}, row {problem['row']}: "
                  f"{problem['message']}")

    speed = float(LaunchConfiguration("speed").perform(context))
    step = float(LaunchConfiguration("step").perform(context))
    if speed != 1.0 or step != 0.001:
        world = set_physics(world, speed, step)

    # GzServer runs the server inside a ROS process, so Ctrl-C stops it.
    # Through gz_sim.launch.py the ruby gz wrapper is stopped instead, and
    # the real server is left running, so the next launch finds the world
    # already taken.
    actions = [
        # gzserver ignores Ctrl-C and gets killed, which leaves Fast DDS
        # shared memory ports locked. The next launch can then hang in
        # rclcpp::Node waiting for one, so the sim talks over UDP only.
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
        # The world includes the CAD arena as model://robogame_arena_cad.
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", os.path.join(share, "models")),
        GzServer(world_sdf_file=world, verbosity_level="1"),
    ]

    if partition:
        # Before the server, so it and everything after share it.
        actions.insert(0, SetEnvironmentVariable("GZ_PARTITION", partition))

    gui_cmd = gui_command()

    if gui:
        card = view_card_path()
        if card is None:
            print("[sim.launch] robogame_gui is not built, so no Camera view "
                  "card; the 1 to 4 keys still work")
        else:
            actions.append(AppendEnvironmentVariable(
                "GZ_GUI_PLUGIN_PATH", card))

        config = view_gui_config(card is not None)

        if config is None:
            raise RuntimeError(
                "Could not find Gazebo's default gui.config to add the key "
                "publisher to")

        gui_cmd += ["--gui-config", config]

        actions.append(ExecuteProcess(
            cmd=gui_cmd, name="gazebo_gui", output="screen"))

        # 1 to 4 in the Gazebo window change the view, see fpv_toggle.py.
        # The driven robot first, so 4 looks through the one being driven.
        order = [driven] + [r for r in robot_ids if r != driven]
        actions.append(Node(
            package="robogame_sim",
            executable="fpv_toggle",
            output="screen",
            arguments=["--robots", ",".join(str(r) for r in order),
                       "--world", WORLD_NAME]
                      + ([] if fpv else ["--no-fpv"]),
        ))

    topics = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    remaps = []

    taken = []

    for index, robot_id in enumerate(robot_ids):
        robot = f"robot_{robot_id}"
        asked_x, asked_y, yaw = spawn_pose(index, len(robot_ids))
        x, y, z, yaw, moved = spawn_spot(ground, asked_x, asked_y, yaw,
                                         taken)
        if moved:
            print(f"[sim.launch] {robot} spawns at {x:.2f}, {y:.2f} "
                  f"instead of {asked_x:.2f}, {asked_y:.2f}, which has no "
                  f"room to drive away")
        taken.append((x, y))

        actions.append(Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{robot}",
            output="screen",
            arguments=[
                "-world", WORLD_NAME,
                "-name", robot,
                "-string", robot_sdf(robot_id, fpv=fpv),
                "-x", f"{x:.3f}", "-y", f"{y:.3f}", "-z", f"{z:.3f}",
                "-Y", f"{yaw:.3f}",
            ],
        ))

        actions.append(Node(
            package="robogame_sim",
            executable="sim_firmware",
            namespace=robot,
            output="screen",
            parameters=[{
                "robot_id": robot_id,
                "drive_model": drive_model,
                "use_sim_time": True,
            }],
        ))

        topics += bridge_topics(robot)
        remaps += ["-r", f"/model/{robot}/tf:=/tf"]

    # Tells the robot console (robot_console.py) which arena this is.
    actions.append(Node(
        package="robogame_sim",
        executable="active_arena",
        output="screen",
        parameters=[{
            "layout": layout_path or "cad",
            "source": "gazebo",
        }],
    ))

    actions.append(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="arena_bridge",
        output="screen",
        arguments=topics,
        ros_arguments=remaps,
    ))

    has_arena_perception = package_exists("arena_perception")

    if controller == "joy":
        robot = f"robot_{driven}"

        actions.append(Node(
            package="joy",
            executable="joy_node",
            namespace=robot,
            output="screen",
            parameters=[{
                "device_id": int(LaunchConfiguration(
                    "joy_device_id").perform(context)),
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
            }],
        ))
        actions.append(Node(
            package="robogame_sim",
            executable="racing_teleop",
            namespace=robot,
            output="screen",
            parameters=[{
                "robot_id": driven,
                "motion_device": LaunchConfiguration(
                    "motion_device").perform(context),
            }],
        ))
    elif controller == "hotplug":
        actions.append(Node(
            package="robogame_sim",
            executable="controller_robots",
            output="screen",
            parameters=[{
                "first_robot": driven,
                "robots": robot_ids,
                "layout": layout_path,
                "world": WORLD_NAME,
                "drive_model": drive_model,
                "fpv": fpv,
            }],
        ))
    elif controller == "classic":
        if not has_arena_perception:
            raise RuntimeError(
                "controller:=classic needs arena_perception in this "
                "workspace, run scripts/sync_arena_perception.sh and build "
                "again")

        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("arena_perception"),
                "launch", "controller.launch.py")),
            launch_arguments={
                "robot_id": str(driven),
                "joy_device_id": LaunchConfiguration(
                    "joy_device_id").perform(context),
            }.items(),
        ))
    elif controller != "none":
        raise RuntimeError(
            f"controller must be none, joy, hotplug or classic, not "
            f"{controller}")

    if autonomy:
        if not has_arena_perception:
            raise RuntimeError(
                "autonomy:=true needs arena_perception in this workspace")

        # autonomy_node is written for robot 1 only.
        actions.append(Node(
            package="arena_perception",
            executable="autonomy_node",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ))

    if chase:
        if len(chase) != 2 or chase[0] == chase[1]:
            raise RuntimeError(
                "chase takes two different robot numbers, hunter then target")

        missing = [r for r in chase if r not in robot_ids]
        if missing:
            raise RuntimeError(
                f"chase robots {missing} are not in robots:={robot_ids}")

        if controller != "none" and chase[0] == driven:
            raise RuntimeError(
                f"robot {driven} cannot chase while the controller "
                f"drives it, try chase:={chase[1]},{chase[0]} or "
                f"drive:={chase[1]}")

        actions.append(Node(
            package="robogame_sim",
            executable="chase_node",
            name=f"chase_robot_{chase[0]}",
            output="screen",
            parameters=[{
                "hunter_id": chase[0],
                "target_id": chase[1],
                "layout": layout_path,
                "use_sim_time": True,
            }],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robots", default_value="1",
            description="Comma separated robot numbers to spawn, e.g. 1,2,3"),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="false runs the Gazebo server only"),
        DeclareLaunchArgument(
            "drive_model", default_value="ideal",
            description="ideal follows cmd_vel, firmware copies the ESP32 "
                        "mixing and stall floor"),
        DeclareLaunchArgument(
            "controller", default_value="none",
            description="joy drives the drive robot with racing_teleop "
                        "(R2 throttle, L2 reverse, Options for wheel "
                        "steering), classic with the repo's "
                        "controller_input. hotplug gives it the first "
                        "controller and spawns a robot for each one "
                        "connected after, sim only"),
        DeclareLaunchArgument(
            "drive", default_value="",
            description="Robot number the controller drives, the first in "
                        "robots when empty"),
        DeclareLaunchArgument(
            "joy_device_id", default_value="0",
            description="Joystick index for controller:=joy"),
        DeclareLaunchArgument(
            "motion_device", default_value="",
            description="Motion sensor js device for wheel steering, found "
                        "by itself when empty"),
        DeclareLaunchArgument(
            "autonomy", default_value="false",
            description="true starts arena_perception autonomy_node, which "
                        "drives robot 1 to points on /robot_1/goal"),
        DeclareLaunchArgument(
            "chase", default_value="",
            description="HUNTER,TARGET, e.g. 1,2, starts chase_node so the "
                        "first robot drives after the second"),
        DeclareLaunchArgument(
            "layout", default_value="default",
            description="Arena layout from arena_editor, by name or path; "
                        "cad uses the Onshape export as it is"),
        DeclareLaunchArgument(
            "partition", default_value="",
            description="Gazebo partition to run in, so a second sim can "
                        "run beside one already going; empty keeps "
                        "GZ_PARTITION as it is"),
        DeclareLaunchArgument(
            "speed", default_value="1",
            description="Real time factor the sim runs at: 3 runs three "
                        "times faster than real time where the computer "
                        "keeps up, 0 as fast as it can. Every node runs on "
                        "sim time, so a chase goes as it would at 1"),
        DeclareLaunchArgument(
            "step", default_value="0.001",
            description="Physics step in seconds. 0.002 about halves the "
                        "work per sim second, so with speed:=0 a headless "
                        "test runs two to three times faster"),
        DeclareLaunchArgument(
            "fpv", default_value="true",
            description="Mounts an FPV camera on the front of every robot. "
                        "4 in the Gazebo window toggles the view through "
                        "it, F1 to F9 pick the robot. false leaves the "
                        "cameras out; 1 to 3 still change the view"),
        OpaqueFunction(function=launch_setup),
    ])
