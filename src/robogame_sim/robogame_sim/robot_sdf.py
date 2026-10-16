"""SDF for one arena robot, with topics named after its robot number.

The chassis and wheels follow ROBOGAME-2.0/src/arena_sim/models/robot_1. The
arm follows robogame-esp: a shoulder and an elbow servo, and a gripper
servo that opens and shuts two fingers.

Topics on the Gazebo side, for robot N:

    /robot_N/sim/cmd_vel     drive, from sim_firmware
    /robot_N/shoulder_cmd    shoulder angle in radians, from sim_firmware
    /robot_N/elbow_cmd       elbow angle in radians
    /robot_N/gripper_cmd     finger opening in radians
    /model/robot_N/odometry  true pose, standing in for the arena camera
    /robot_N/wheel_odom      wheel odometry, which drifts as wheels slip
    /robot_N/joint_states    every joint
"""

import sys

CHASSIS = (0.08, 0.05, 0.03)
WHEEL_RADIUS = 0.02
WHEEL_WIDTH = 0.01
WHEEL_X = 0.03
WHEEL_Y = 0.03

SHOULDER = (0.035, 0.0, 0.018)
UPPER_ARM_LENGTH = 0.05
FOREARM_LENGTH = 0.04
FINGER_LENGTH = 0.025
FINGER_GAP = 0.006

# One colour per team, robot 0 is the instructor's.
COLOURS = [
    (0.9, 0.9, 0.9),
    (0.9, 0.15, 0.15),
    (0.15, 0.45, 0.95),
    (0.15, 0.75, 0.25),
    (0.95, 0.75, 0.1),
    (0.65, 0.25, 0.85),
    (0.95, 0.45, 0.1),
]


def box_inertia(mass, x, y, z):
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def inertial(mass, inertia, pose="0 0 0 0 0 0"):
    ixx, iyy, izz = inertia

    return f"""
      <inertial>
        <pose>{pose}</pose>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx:.9f}</ixx><iyy>{iyy:.9f}</iyy><izz>{izz:.9f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>"""


def material(rgb):
    r, g, b = rgb

    return f"""
          <material>
            <ambient>{r * 0.8:.3f} {g * 0.8:.3f} {b * 0.8:.3f} 1</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>"""


def box_link(name, pose, size, mass, rgb, offset="0 0 0 0 0 0"):
    x, y, z = size
    geometry = f"<box><size>{x} {y} {z}</size></box>"

    return f"""
    <link name="{name}">
      <pose>{pose}</pose>{inertial(mass, box_inertia(mass, x, y, z), offset)}
      <collision name="collision">
        <pose>{offset}</pose>
        <geometry>{geometry}</geometry>
      </collision>
      <visual name="visual">
        <pose>{offset}</pose>
        <geometry>{geometry}</geometry>{material(rgb)}
      </visual>
    </link>"""


def wheel_link(name, x, y, z):
    mass = 0.02
    side = mass * (3 * WHEEL_RADIUS ** 2 + WHEEL_WIDTH ** 2) / 12.0
    spin = mass * WHEEL_RADIUS ** 2 / 2.0
    geometry = (
        f"<cylinder><radius>{WHEEL_RADIUS}</radius>"
        f"<length>{WHEEL_WIDTH}</length></cylinder>"
    )

    # The cylinder is turned onto its side, so the link frame stays square
    # with the chassis and the axle is the link's Y axis. In the collision
    # frame the axle is Z, which is what fdir1 refers to.
    return f"""
    <link name="{name}">
      <pose>{x} {y} {z} 0 0 0</pose>{inertial(mass, (side, spin, side))}
      <collision name="collision">
        <pose>0 0 0 1.5708 0 0</pose>
        <geometry>{geometry}</geometry>
        <surface>
          <friction>
            <ode>
              <!-- Sideways along the axle, low so a four wheel robot can
                   skid round a turn -->
              <mu>0.1</mu>
              <!-- Rolling direction -->
              <mu2>0.9</mu2>
              <fdir1>0 0 1</fdir1>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <pose>0 0 0 1.5708 0 0</pose>
        <geometry>{geometry}</geometry>{material((0.08, 0.08, 0.08))}
      </visual>
    </link>"""


def revolute(name, parent, child, axis, lower, upper, velocity, effort,
             pose="0 0 0 0 0 0"):
    return f"""
    <joint name="{name}" type="revolute">
      <pose>{pose}</pose>
      <parent>{parent}</parent>
      <child>{child}</child>
      <axis>
        <xyz>{axis}</xyz>
        <limit>
          <lower>{lower}</lower>
          <upper>{upper}</upper>
          <effort>{effort}</effort>
          <velocity>{velocity}</velocity>
        </limit>
        <dynamics><damping>0.0005</damping></dynamics>
      </axis>
    </joint>"""


def position_controller(robot, joint, topic):
    # use_velocity_commands moves the joint like a hobby servo, at the
    # joint's velocity limit, and stays steady with links this light.
    return f"""
    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>{joint}</joint_name>
      <topic>/{robot}/{topic}</topic>
      <use_velocity_commands>true</use_velocity_commands>
      <p_gain>20</p_gain>
    </plugin>"""


def robot_sdf(robot_id):
    robot = f"robot_{robot_id}"
    colour = COLOURS[robot_id % len(COLOURS)]
    arm_colour = (0.25, 0.25, 0.28)

    sx, sy, sz = SHOULDER
    elbow_x = sx + UPPER_ARM_LENGTH
    wrist_x = elbow_x + FOREARM_LENGTH

    links = [
        box_link("chassis", "0 0 0 0 0 0", CHASSIS, 0.5, colour),
        wheel_link("left_front_wheel", WHEEL_X, WHEEL_Y, 0),
        wheel_link("left_rear_wheel", -WHEEL_X, WHEEL_Y, 0),
        wheel_link("right_front_wheel", WHEEL_X, -WHEEL_Y, 0),
        wheel_link("right_rear_wheel", -WHEEL_X, -WHEEL_Y, 0),
        box_link(
            "upper_arm", f"{sx} {sy} {sz} 0 0 0",
            (UPPER_ARM_LENGTH, 0.008, 0.006), 0.01, arm_colour,
            offset=f"{UPPER_ARM_LENGTH / 2} 0 0 0 0 0"),
        box_link(
            "forearm", f"{elbow_x} {sy} {sz} 0 0 0",
            (FOREARM_LENGTH, 0.008, 0.006), 0.008, arm_colour,
            offset=f"{FOREARM_LENGTH / 2} 0 0 0 0 0"),
        box_link(
            "left_finger", f"{wrist_x} {FINGER_GAP} {sz} 0 0 0",
            (FINGER_LENGTH, 0.003, 0.012), 0.002, colour,
            offset=f"{FINGER_LENGTH / 2} 0 0 0 0 0"),
        box_link(
            "right_finger", f"{wrist_x} {-FINGER_GAP} {sz} 0 0 0",
            (FINGER_LENGTH, 0.003, 0.012), 0.002, colour,
            offset=f"{FINGER_LENGTH / 2} 0 0 0 0 0"),
    ]

    wheel = dict(lower=-1e16, upper=1e16, velocity=40, effort=2.0)
    joints = [
        revolute("left_front_wheel_joint", "chassis", "left_front_wheel",
                 "0 1 0", **wheel),
        revolute("left_rear_wheel_joint", "chassis", "left_rear_wheel",
                 "0 1 0", **wheel),
        revolute("right_front_wheel_joint", "chassis", "right_front_wheel",
                 "0 1 0", **wheel),
        revolute("right_rear_wheel_joint", "chassis", "right_rear_wheel",
                 "0 1 0", **wheel),
        # About -Y, so a positive angle lifts the arm.
        revolute("shoulder_joint", "chassis", "upper_arm", "0 -1 0",
                 -1.5708, 1.5708, velocity=5.0, effort=1.0),
        revolute("elbow_joint", "upper_arm", "forearm", "0 -1 0",
                 -1.5708, 1.5708, velocity=5.0, effort=1.0),
        # Mirrored, so one positive command opens both fingers.
        revolute("left_finger_joint", "forearm", "left_finger", "0 0 1",
                 0, 0.7, velocity=5.0, effort=0.5),
        revolute("right_finger_joint", "forearm", "right_finger", "0 0 -1",
                 0, 0.7, velocity=5.0, effort=0.5),
    ]

    plugins = [
        f"""
    <plugin filename="gz-sim-diff-drive-system"
            name="gz::sim::systems::DiffDrive">
      <left_joint>left_front_wheel_joint</left_joint>
      <left_joint>left_rear_wheel_joint</left_joint>
      <right_joint>right_front_wheel_joint</right_joint>
      <right_joint>right_rear_wheel_joint</right_joint>
      <wheel_separation>{2 * WHEEL_Y}</wheel_separation>
      <wheel_radius>{WHEEL_RADIUS}</wheel_radius>
      <topic>/{robot}/sim/cmd_vel</topic>
      <odom_topic>/{robot}/wheel_odom</odom_topic>
      <tf_topic>/{robot}/wheel_odom/tf</tf_topic>
      <frame_id>{robot}/odom</frame_id>
      <child_frame_id>{robot}/base_link</child_frame_id>
      <odom_publish_frequency>30</odom_publish_frequency>
    </plugin>""",
        f"""
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_topic>/model/{robot}/odometry</odom_topic>
      <tf_topic>/model/{robot}/tf</tf_topic>
      <odom_frame>arena</odom_frame>
      <robot_base_frame>{robot}/base_link</robot_base_frame>
      <odom_publish_frequency>30</odom_publish_frequency>
      <dimensions>2</dimensions>
    </plugin>""",
        f"""
    <plugin filename="gz-sim-joint-state-publisher-system"
            name="gz::sim::systems::JointStatePublisher">
      <topic>/{robot}/joint_states</topic>
    </plugin>""",
        position_controller(robot, "shoulder_joint", "shoulder_cmd"),
        position_controller(robot, "elbow_joint", "elbow_cmd"),
        position_controller(robot, "left_finger_joint", "gripper_cmd"),
        position_controller(robot, "right_finger_joint", "gripper_cmd"),
    ]

    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{robot}">
{''.join(links)}
{''.join(joints)}
{''.join(plugins)}
  </model>
</sdf>
"""


def main():
    # Print one robot, handy for checking a model with gz sdf -k.
    robot_id = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    print(robot_sdf(robot_id))


if __name__ == "__main__":
    main()
