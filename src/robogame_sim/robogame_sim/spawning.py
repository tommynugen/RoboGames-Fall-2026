"""Where robots spawn in the arena, and the Gazebo topics each one bridges.

Shared by sim.launch.py, which spawns the robots it is asked for, and
controller_robots, which spawns one for each controller connected later.
"""

import math

from robogame_sim import arena_layout

# Spawn spots on a circle, every robot facing the centre.
SPAWN_RADIUS = 0.35
SPAWN_HEIGHT = 0.025
# Clear ground a robot wants in front of it at the start.
CLEAR_AHEAD = 0.25


def spawn_pose(index, count):
    angle = -3 * math.pi / 4 + 2 * math.pi * index / max(count, 1)

    return (
        SPAWN_RADIUS * math.cos(angle),
        SPAWN_RADIUS * math.sin(angle),
        angle + math.pi,
    )


def spawn_spot(ground, x, y, yaw, taken=()):
    """x, y, z, yaw to spawn a robot asked for at x, y, and whether it was
    moved. Off any module and clear of it, so the robot can drive away,
    apart from the robots in taken, and on top of whatever is there rather
    than inside it."""
    spot = arena_layout.open_spot(ground, x, y, taken)
    moved = spot != (x, y)

    if moved:
        x, y = spot
        yaw = math.atan2(-y, -x)

    yaw = open_heading(ground, x, y, yaw)
    under = ground.height_at(x, y)
    z = SPAWN_HEIGHT + (0.0 if math.isnan(under) else max(0.0, under))
    return x, y, z, yaw, moved


def clear_ahead(ground, x, y, yaw, step=0.01, reach=1.0):
    """How far a robot at x, y can drive along yaw before ground it cannot
    drive on."""
    distance = 0.0

    while distance < reach:
        cell = ground.cell(x + (distance + step) * math.cos(yaw),
                           y + (distance + step) * math.sin(yaw))
        if not ground.inside(cell) or not ground.free[cell]:
            break
        distance += step

    return distance


def open_heading(ground, x, y, yaw, room=CLEAR_AHEAD):
    """yaw, if there is room to drive off along it, else the way of eight
    with the most. The arm reaches past the chassis, so a robot facing a
    module close by is pinned against it."""
    if clear_ahead(ground, x, y, yaw) >= room:
        return yaw

    headings = [yaw + k * math.pi / 4 for k in range(8)]
    return max(headings, key=lambda h: clear_ahead(ground, x, y, h))


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
