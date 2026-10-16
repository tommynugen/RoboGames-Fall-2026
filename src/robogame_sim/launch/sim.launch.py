"""Gazebo arena with one simulated robot per robot number.

    ros2 launch robogame_sim sim.launch.py
    ros2 launch robogame_sim sim.launch.py robots:=1,2,3
    ros2 launch robogame_sim sim.launch.py controller:=joy
    ros2 launch robogame_sim sim.launch.py autonomy:=true
    ros2 launch robogame_sim sim.launch.py robots:=1,2 chase:=1,2
    ros2 launch robogame_sim sim.launch.py drive_model:=firmware gui:=false

Each robot listens on /robot_N/cmd_vel and /robot_N/arm_command, the same
topics microcontroller_node takes, so do not run the BLE bridge alongside.
"""

import glob
import math
import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_sim.actions import GzServer

from robogame_sim.robot_sdf import robot_sdf

WORLD_NAME = "robogame_arena"

# Spawn spots on a circle, every robot facing the centre.
SPAWN_RADIUS = 0.35
SPAWN_HEIGHT = 0.025


def spawn_pose(index, count):
    angle = -3 * math.pi / 4 + 2 * math.pi * index / max(count, 1)

    return (
        SPAWN_RADIUS * math.cos(angle),
        SPAWN_RADIUS * math.sin(angle),
        angle + math.pi,
    )


def bridge_topics(robot):
    return [
        f"/{robot}/sim/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        f"/{robot}/shoulder_cmd@std_msgs/msg/Float64]gz.msgs.Double",
        f"/{robot}/elbow_cmd@std_msgs/msg/Float64]gz.msgs.Double",
        f"/{robot}/gripper_cmd@std_msgs/msg/Float64]gz.msgs.Double",
        f"/model/{robot}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"/{robot}/wheel_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"/{robot}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        f"/model/{robot}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
    ]


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


def package_exists(name):
    try:
        get_package_share_directory(name)
    except PackageNotFoundError:
        return False

    return True


def launch_setup(context):
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

    share = get_package_share_directory("robogame_sim")
    world = os.path.join(share, "worlds", f"{WORLD_NAME}.sdf")

    # GzServer runs the server inside a ROS process, so Ctrl-C stops it.
    # Through gz_sim.launch.py the ruby gz wrapper is stopped instead, and
    # the real server is left running, so the next launch finds the world
    # already taken.
    actions = [GzServer(world_sdf_file=world, verbosity_level="1")]

    if gui:
        actions.append(ExecuteProcess(
            cmd=gui_command(), name="gazebo_gui", output="screen"))

    topics = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    remaps = []

    for index, robot_id in enumerate(robot_ids):
        robot = f"robot_{robot_id}"
        x, y, yaw = spawn_pose(index, len(robot_ids))

        actions.append(Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{robot}",
            output="screen",
            arguments=[
                "-world", WORLD_NAME,
                "-name", robot,
                "-string", robot_sdf(robot_id),
                "-x", f"{x:.3f}", "-y", f"{y:.3f}", "-z", str(SPAWN_HEIGHT),
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

    actions.append(Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="arena_bridge",
        output="screen",
        arguments=topics,
        ros_arguments=remaps,
    ))

    first_robot = robot_ids[0]
    has_arena_perception = package_exists("arena_perception")

    if controller == "joy":
        if not has_arena_perception:
            raise RuntimeError(
                "controller:=joy needs arena_perception in this workspace, "
                "run scripts/sync_arena_perception.sh and build again")

        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("arena_perception"),
                "launch", "controller.launch.py")),
            launch_arguments={
                "robot_id": str(first_robot),
                "joy_device_id": LaunchConfiguration(
                    "joy_device_id").perform(context),
            }.items(),
        ))
    elif controller != "none":
        raise RuntimeError(f"controller must be none or joy, not {controller}")

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

        actions.append(Node(
            package="robogame_sim",
            executable="chase_node",
            name=f"chase_robot_{chase[0]}",
            output="screen",
            parameters=[{
                "hunter_id": chase[0],
                "target_id": chase[1],
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
            description="joy starts joy_node and controller_input for the "
                        "first robot"),
        DeclareLaunchArgument(
            "joy_device_id", default_value="0",
            description="Joystick index for controller:=joy"),
        DeclareLaunchArgument(
            "autonomy", default_value="false",
            description="true starts arena_perception autonomy_node, which "
                        "drives robot 1 to points on /robot_1/goal"),
        DeclareLaunchArgument(
            "chase", default_value="",
            description="HUNTER,TARGET, e.g. 1,2, starts chase_node so the "
                        "first robot drives after the second"),
        OpaqueFunction(function=launch_setup),
    ])
