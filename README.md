# Robogame Gazebo sim (private test)

A private workspace for trying Gazebo with the ROBOGAME-2.0 nodes. It lives
outside every git checkout and has no remote, so nothing here reaches
GitHub. The ROBOGAME-2.0 checkout is only ever read.

```
src/robogame_sim       arena world, robot model, sim_firmware, launch
src/arena_perception   copy of the repo package, made by the sync script
scripts/sync_arena_perception.sh
```

## What it simulates

`sim_firmware` takes the place of `microcontroller_node` and the ESP32 for
one robot. It listens on the same topics the BLE bridge does:

| Topic | Type | |
| --- | --- | --- |
| `/robot_N/cmd_vel` | `geometry_msgs/Twist` | from `controller_input` or `autonomy_node` |
| `/robot_N/arm_command` | `std_msgs/String` | `+ - X H o c`, as in PROTOCOL.md |

and publishes what the robot does:

| Topic | Type | |
| --- | --- | --- |
| `/model/robot_N/odometry` | `nav_msgs/Odometry` | true pose, standing in for the arena camera. `autonomy_node` reads this |
| `/robot_N/wheel_odom` | `nav_msgs/Odometry` | wheel odometry, which drifts as the four wheels skid |
| `/robot_N/joint_states` | `sensor_msgs/JointState` | wheels, shoulder, elbow, fingers |
| `/tf` | | `arena` to `robot_N/base_link` |

Like the firmware, the robot stops if no drive command arrives for 750 ms,
and each arm command moves the shoulder or elbow 5 degrees, clamped to 0 to
180 degrees, starting from 90.

`drive_model` picks how `cmd_vel` turns into wheel motion:

- `ideal` (default) follows the Twist, clamped to the bridge's full speeds
  of 0.5 m/s and 1.5 rad/s. Use it to work on autonomy.
- `firmware` copies the real path: the bridge turns the Twist into
  percentages, the firmware mixes them into left and right and lifts any
  non-zero side to the 35 percent stall floor. Wheel speed is taken as
  linear in duty, with `wheel_top_speed` (0.5 m/s, a guess) at 100 percent.
  A gentle curve such as 0.1 m/s and 0.3 rad/s becomes a near pivot at
  about 5 rad/s, which is how a real robot will react too.

## Setup

ROS 2 Lyrical is installed here, not the Jazzy the repo targets. Gazebo
comes with `ros_gz`:

```
sudo apt install ros-lyrical-ros-gz
```

Then build:

```
cd ~/robogame-sim
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

To pick up changes from the repo, run `scripts/sync_arena_perception.sh`
and build again.

## Run

```
ros2 launch robogame_sim sim.launch.py                    # robot 1
ros2 launch robogame_sim sim.launch.py robots:=0,1,2,3    # several robots
ros2 launch robogame_sim sim.launch.py gui:=false         # no window
ros2 launch robogame_sim sim.launch.py drive_model:=firmware
```

Drive with a PS4 controller, through the real `controller_input`:

```
ros2 launch robogame_sim sim.launch.py controller:=joy
```

Or from the keyboard, in a second terminal:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/robot_1/cmd_vel -p stamped:=false
```

Move the arm by hand:

```
ros2 topic pub --once /robot_1/arm_command std_msgs/msg/String "{data: '+++'}"
ros2 topic pub --once /robot_1/arm_command std_msgs/msg/String "{data: 'c'}"
```

Send `autonomy_node` to a point (arena centre is 0, 0, walls at 0.5):

```
ros2 launch robogame_sim sim.launch.py autonomy:=true
ros2 topic pub --once /robot_1/goal geometry_msgs/msg/Point "{x: 0.2, y: 0.3}"
```

## Tests

```
cd src/robogame_sim && python3 -m pytest test
```

These check the firmware maths against motors.cpp and arm.cpp, and that the
robot model is valid XML. Print a robot's SDF with `ros2 run robogame_sim
robot_sdf 2`, and check it with `gz sdf -k` once Gazebo is installed.

## Differences from the repo's arena_sim

- The world is the same arena, cleaned so it parses. The original has a
  stray `-----` line in the world and `<! --- JOINTS --- >` in
  `robot_1/model.sdf`, and neither is valid XML.
- Robots are spawned by the launch file from a template, one per robot
  number, each with its own colour and topics.
- The model has the shoulder, elbow and gripper from the firmware in place
  of the single lifting arm.
- Wheel links stay square with the chassis and only the cylinder is turned,
  so the wheel axle is plainly the link's Y axis.

## Not done yet

- No overhead camera or AprilTags. `apriltag_node` and `gazebo_viz_node`
  still need the real camera.
- Arm lengths and the servo angle that counts as level are guesses. Measure
  a real robot and change `robot_sdf.py`.
- `wheel_top_speed` is a guess. Time a real robot across the arena at full
  speed.
