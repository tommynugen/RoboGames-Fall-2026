# Robogame testing fork: Gazebo sim (private test)

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

ROS 2 Lyrical is installed here, not the Jazzy the repo targets, so this
runs Gazebo Jetty (gz-sim 10). Gazebo comes with `ros_gz`:

```
sudo apt install ros-lyrical-ros-gz
```

Then build:

```
cd ~/Documents/PlatformIO/Projects/robogame-testing-fork
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

Drive with a PS4 or PS5 controller, like a racing game:

```
ros2 launch robogame_sim sim.launch.py controller:=joy
```

| | Stick mode | Wheel mode |
| --- | --- | --- |
| R2 | throttle | throttle |
| L2 | reverse | reverse |
| Steer | left stick | roll the controller like a steering wheel |
| Shoulder | Triangle / Cross | tip the controller up / down, or Triangle / Cross |
| Elbow | Circle / Square | Circle / Square |
| Gripper | R1 open, L1 close | R1 open, L1 close |
| Share | | re-centre the wheel |

Options switches mode, and `/robot_N/steering_mode` says which is on. Wheel
mode takes the way the controller is held when it starts as straight ahead
and level, rolling 45 degrees is full lock, and tipping 20 degrees moves the
shoulder. Both triggers are proportional, and as in a car, steering while
reversing swings the back of the robot that way. With no throttle the
stick spins the robot in place.

This is `racing_teleop` in this package, not the repo's `controller_input`,
so a sync does not undo it. It reads `/robot_N/joy` from `joy_node` and the
controller's motion sensor from its own `/dev/input/jsN` ("... Motion
Sensors"), found by itself or set with `motion_device:=/dev/input/js2`. If
the wheel steers or tips the arm the wrong way, add
`-p wheel_steer_inverted:=true` or `-p wheel_arm_inverted:=true`, and
`-p wheel_debug:=true` logs the angles. The repo's controls are still there
as `controller:=classic`.

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

## Chase another robot

`chase_node` drives one robot after another. It reads both robots' true
poses from `/model/robot_N/odometry` and publishes the hunter's `cmd_vel`:

```
ros2 launch robogame_sim sim.launch.py robots:=1,2 chase:=1,2
```

Robot 1 hunts robot 2. Drive robot 2 from the keyboard to make it run:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/robot_2/cmd_vel -p stamped:=false
```

Or be chased: the controller drives the first robot in `robots`, so have
robot 2 hunt it:

```
ros2 launch robogame_sim sim.launch.py robots:=1,2 controller:=joy chase:=2,1
```

The launch refuses a hunter that the controller is also driving, and any
misspelt argument (`controler:=joy` would otherwise quietly start no
controller).

The hunter predicts where the target is going. From the target's poses it
measures its speed and turn rate, assumes it keeps both, and so follows a
robot driving an arc round the arc rather than off its tangent. Along that
path it finds the earliest point it can reach in time, counting its own
turning and driving at `intercept_speed` (0.25 m/s), and heads there. A
target that is still or close is aimed at directly, a fast or far one is
led by up to `prediction_horizon` (1.5 s), and predicted points stay 6 cm
inside the edge of the floor. Once it has caught up, the hunter adds the target's speed to its
own so it keeps pace instead of trailing, fading that out over the last
2 cm so a target that stops is not run into.

Forward speed falls off with heading error, and the hunter turns on the
spot if its aim point is more than about 57 degrees off its nose. It stops
with its fingertips `clearance` (3 cm) short of the target. The arm reaches
15 cm past the centre, so the stopping distance depends on which way the
target faces: 0.33 m between centres head on, about 0.22 m from behind. If
it ends up too close, it backs off. If either pose is older than 0.5 s, it
stops. Speeds, gains and the prediction are parameters (see the top of
`chase_node.py`).

The hunter knows the arena's terrain and plans its way to the target
(`terrain.py`). At start up it builds the arena from the same layout as the
sim (see Arena layouts below; `layout:=cad` reads
`models/robogame_arena_cad/model.sdf` instead) and turns its collision
geometry into a 2 cm height map. Wherever the height jumps between
cells, at a platform's edge, a ramp's sides or the edge of the floor
tiles, is a step the wheels cannot climb, and the hunter's centre is kept
5.3 cm from it, enough to turn on the spot. On and within 7 cm of sloped
ground it may only drive within 15 degrees of the slope, so it lines up
before a ramp and goes straight up or down it. On a ramp it keeps to at
least `ramp_speed` (0.15 m/s) going forward, so it does not stall on the
steep top, and holds still rather than backing down it. Nothing about the
layout is written into the code.

Every control step it checks whether it can drive straight at the target.
If so it chases as above. If not, say the platform is in the way or the
target is up on it, it plans the cheapest path over the height map with
A*, pulls it straight, and drives it, slowing before sharp corners such as
the turn at the foot of a ramp. So it goes round the near side of a
platform to a target behind it, takes the ramp up to a target on top, and
comes back down the ramp after a target that has left. A target it
cannot get to, on a platform no ramp leads to, it waits for at the
nearest place it can reach. It takes about 15 ms a step.

Checked headless on 2026-10-16 with the CAD arena, robot 2 still, robot 1
placed by hand. Height is the true height of robot 1 from Gazebo:

| Robot 1 from | Robot 2 at | What robot 1 did |
| --- | --- | --- |
| floor north of the platform | on the platform | round the east side, lined up in front of the ramp, up it (height +2.54 cm), stopped 0.23 m from robot 2 at 9 s |
| floor east of the platform | on the platform | round the south east corner and up the ramp, stopped 0.22 m away at 6 s |
| on the platform | floor north of it | down the ramp (height -2.53 cm), round the east side, stopped 0.23 m away at 10 s |
| floor west of the platform | floor east of it | round the north side on the floor, stopped 0.24 m away at 6 s |

Checked headless on 2026-10-16, robot 2 steered round a 0.3 m orbit of the
arena from the start, robot 1 hunting it from 0.7 m away. Gap is distance
past the stopping distance, measured after the first 5 s:

| Target | Before prediction | With prediction |
| --- | --- | --- |
| 0.15 m/s | reached 1.9 s, gap 6.2 cm, engaged 4% | reached 1.7 s, gap 1.0 cm, engaged 100% |
| 0.22 m/s | reached 2.2 s, gap 8.5 cm, engaged 0% | reached 1.3 s, gap 0.9 cm, engaged 100% |
| 0.22 m/s, 3 s go 2 s stop | gap 5.6 cm, engaged 45% | gap 0.6 cm, engaged 100%, closest -0.2 cm |

Before, the hunter sat behind the target and never caught it. A still
target is still approached and held 1 cm outside the stopping distance. The
maths is covered by `test/test_chase.py`, the terrain and paths by
`test/test_terrain.py`.

## Arena layouts

The arena is a 12 x 12 grid of 4 inch (10.16 cm) floor tiles, 48 inches
square, and the modules of the CAD's MODULES part studio stand on them,
moved about between games:

| Module | In the layout | |
|---|---|---|
| 1" platform | `platform` | what ramps lead up to |
| 2" platform | `platform_2` | too high for a ramp, so an obstacle |
| Curved ramp | `ramp` | rises 1 inch over one square |
| Wall | `wall` | 1 x 4 inches, 2 high, pegged on one side of a square |
| Thwomp | `thwomp` | 1 x 4 inches, 6 high, pegged like a wall; fixed in the sim |

Map how they are set up on the table in the layout editor:

```
ros2 run robogame_sim arena_editor
```

It opens http://localhost:8765 in the browser (only this computer can
reach it; `--port` and `--no-browser` change that). Pick a module (keys 1
to 5) and click or drag on the grid, which is seen from above with +x to
the right and +y up, each square drawn with its tile's four peg holes.
Click a ramp, wall or thwomp with its own tool to turn it a quarter turn,
or press R over it. A ramp's arrow points uphill; a wall or thwomp stands
on the line of peg holes an inch from the middle, on the side it faces.
Right-click or the Floor tool (6) clears a square, and Ctrl+Z undoes. As
you edit it checks the layout and outlines in red any ramp whose low end
runs into the edge, a module or a wall facing it, any ramp that leads to
no 1" platform, any 1" platform no robot can get onto from the floor, and
any 1" platform too narrow to drive on. A robot needs about 5.3 cm
clear of every edge, so platforms and ramps have to be at least two
modules wide. Name the layout and Save, and it is written to
`src/robogame_sim/layouts/NAME.json`, one module per line so it diffs
well. `http://localhost:8765/#NAME` opens a saved one directly.

Layouts made for the old 9 x 9 grid of 5 inch modules still open, with a
warning; pick 12 x 12 under Grid to grow one about the middle and Save.

Then run the sim on it, no rebuild needed:

```
ros2 launch robogame_sim sim.launch.py layout:=NAME robots:=1,2 chase:=1,2
```

The launch builds the world from the layout with the meshes and collision
shapes `scripts/export_onshape_arena.py` exported from the CAD, so a
layout looks and collides like the real modules, prints the same checks,
spawns robots on top of whatever is under their spawn spot, lifts the
tokens onto it too, and hands the layout to `chase_node` so the planner
and the sim agree. Walls and thwomps are obstacles to the planner like a
platform's edge. `layout:=default` (the default) is a 4 x 4 platform in
the middle with four ramps on its -y side. `layout:=cad` uses the Onshape
export as it is, MAIN ASSEMBLY's modules where it puts them. `rotation`
in the file is degrees anticlockwise from above: a ramp at 0 rises toward
+y, 90 toward -x, 180 toward -y, 270 toward +x, and a wall or thwomp at
those stands on the +y, -x, -y or +x side of its square.

Robots spawn on a circle round the middle, and one whose spot is on a
ramp or wedged against a module (closer than 8 cm, as at the corners of
the default platform) is moved to the nearest open, level ground at the
same height, and the launch prints where.

Only one sim can run at a time in a Gazebo partition, since they share the
world's name. If one is already running the launch stops straight away and
says so; before, the second one looked frozen, as its windows and nodes
attached to the first. Stop the first, or add `partition:=NAME` to run the
second beside it.

After changing the modules in Onshape, run
`scripts/export_onshape_arena.py` and rebuild. It exports every part of
the MODULES part studio, and the layouts place each by its mesh, so a
module that changes size or height needs no code change; a new kind of
module needs an entry in `PARTS` in `arena_layout.py` and a tool in
`arena_editor.html`.

Checked headless on 2026-10-16 with the 4 inch modules, on a layout of a
3 x 3 1" platform with three ramps on its -x side, two 2" platforms, two
walls and a thwomp: the world loaded every mesh, robot 2 spawned on top of
the platform (z 0.045 m), and robot 1 went round to the ramps and up them
(z 0.045 m) and held 0.24 m from robot 2. The editor was checked in
headless Chromium, in light and dark, with a layout of every module and
with an old 9 x 9 layout.

## Tests

```
cd src/robogame_sim && python3 -m pytest test
```

These check the firmware maths against motors.cpp and arm.cpp, the racing
controls and steering wheel maths, that the robot model is valid XML, the
chase and path planning, and the arena layouts and their editor. Print a robot's SDF with `ros2 run robogame_sim
robot_sdf 2`, and check it with `gz sdf -k` once Gazebo is installed.

## Checked in Gazebo

Run headless on 2026-10-16:

- Spawning one robot or four (`robots:=0,1,2,3`), each at its own spot.
- Driving: 0.2 m/s for about 2.7 s covered 0.54 m, then the 750 ms
  timeout stopped the robot.
- Turning, true pose against command: a 1.5 rad/s spin for 2 s turned 183
  degrees for 172 asked, and a 0.2 m/s, 0.5 rad/s arc for 3 s turned 87
  for 86. A spin in place drifts about 5 cm. Wheel odometry stayed within
  about 10 degrees of the truth.
- Arm: `+++++` gave shoulder +25 degrees, `HHHH` elbow -20, `c` shut the
  fingers and `o` opened them.
- The repo's unchanged `autonomy_node` reached (0.2, 0.3) and then
  (-0.3, -0.3), stopping within its 5 cm tolerance.
- `drive_model:=firmware`: 0.1 m/s with 0.3 rad/s pivoted about 290 degrees
  in a second.
- The GUI opens, and Ctrl-C stops everything, server and window included.

## Known issues

- `parameter_bridge` segfaults (exit -11) as the launch shuts down. It
  happens when the Gazebo server stops before the bridge, which also
  happens running the bridge alone with no launch, so it is a bug in the
  Lyrical `ros_gz_bridge` 3.0.10 rather than here. It only happens at exit
  and does no harm.
- If an older run was killed hard, a `gz-sim-main` or `gz-sim-gui-client`
  can be left behind and the next launch reports "Another world of the
  same name is running". Stop it with `pkill -x gz-sim-main` or
  `pkill -f gz-sim-gui-client`.

## Differences from the repo's arena_sim

- The world is the same arena, cleaned so it parses. The original has a
  stray `-----` line in the world and `<! --- JOINTS --- >` in
  `robot_1/model.sdf`, and neither is valid XML.
- Robots are spawned by the launch file from a template, one per robot
  number, each with its own colour and topics.
- The model has the shoulder, elbow and gripper from the firmware in place
  of the single lifting arm.
- Gazebo is started with `ros_gz_sim`'s `GzServer` action and the GUI
  client directly. Going through `gz_sim.launch.py` stops only the `gz`
  ruby wrapper on Ctrl-C, and leaves the real server running.
- Wheel links stay square with the chassis and only the cylinder is turned,
  so the wheel axle is plainly the link's Y axis.

## Not done yet

- No overhead camera or AprilTags. `apriltag_node` and `gazebo_viz_node`
  still need the real camera.
- Arm lengths and the servo angle that counts as level are guesses. Measure
  a real robot and change `robot_sdf.py`.
- `wheel_top_speed` is a guess. Time a real robot across the arena at full
  speed.
