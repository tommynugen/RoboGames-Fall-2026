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
ros2 launch robogame_sim sim.launch.py fpv:=false         # no onboard camera
ros2 launch robogame_sim sim.launch.py gui:=false fpv:=false speed:=0 step:=0.002
```

The last runs a headless test as fast as the computer goes. `speed` is the
real time factor (1 by default, 0 as fast as it can) and `step` the physics
step (0.001 s by default). The arena's meshes keep Gazebo at about real
time with 1 ms steps however high `speed` is; with 2 ms steps a two robot
chase on test4 runs at about twice real time, and goes the same way, since
every node runs on sim time. Separate runs side by side, each with its own
`partition` and `ROS_DOMAIN_ID`, go faster still.

Every robot has an FPV camera low on the front of its chassis, under the
arm, so it does not move when the arm does; `fpv:=false` leaves it out.
Click the Gazebo window's 3D view, then:

| Key | |
|---|---|
| 1 | look straight down on the arena |
| 2 | look over the arena's front (-y) edge, where the window starts |
| 3 | look over the arena's front right corner |
| 4 | look through the driven robot's FPV |
| F1 to F9 | look through that robot's FPV instead, robots spawned for a controller too |
| Esc | leave FPV where the camera is, rather than going back over the arena |

Each key switches to its view; pressed again it does nothing, so a view
you have moved with the mouse stays put. The three views over the arena
come as close as the whole floor fits, with a little room for the walls,
and keep it in the middle of the picture. They fit the 3D view's shape
as the Camera view card reports it, and fit again when the window is
resized.

The Camera view card in the window's side panel does the same with two
drop downs, View and Robot, and follows the keys. It is a Gazebo GUI
plugin, `src/robogame_gui`, built by `colcon build` with the rest; without
it the launch says so and the keys still work. With `fpv:=false` only the
three views over the arena work.

The Gazebo window's own camera is put on the robot (`fpv_toggle.py`), so
nothing extra is rendered. Each change from FPV back to the arena logs an
"Unable to follow target" error from Gazebo: that is how the camera is let
go.

Checked on 2026-10-18 with the window up, robots 1 and 2: keys 1 to 4 and
F1, F2 sent on `/keyboard/keypress`, and views asked for on
`/robogame/view` as the card does, each moved `/gui/camera/pose` to the
view asked for and `/robogame/view/state` followed; the card loaded with
no errors. Tests check each view looks down on the arena with its front
edge at the bottom.

Checked again on 2026-10-18: with the card reporting the 3D view as 577 x
797 pixels (0.724), screenshots of 1, 2 and 3 show the floor across about
95% of the picture and in the middle of it; 1 pressed twice and 4
pressed twice each moved the camera once. Tests check every view fits
the floor to 95% of the tighter side, centred, for tall, square and wide
windows, and that a key pressed again does nothing.

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

The controller drives the first robot in `robots`, or the one `drive:=N`
names.

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

## A robot for every controller

The editor's Test it command, for the sim only:

```
ros2 launch robogame_sim sim.launch.py layout:=NAME robots:=1,2 chase:=1,2 \
    controller:=hotplug drive:=2
```

Robot 1 chases robot 2, and the first controller connected drives robot 2.
Every controller connected after it gets a robot of its own, numbered on
from the highest there (3, 4, ...), spawned on open ground facing somewhere
it can drive, with racing controls and wheel steering from that
controller's own motion sensor. A controller that drops and comes back
drives the same robot again; its robot waits where it stopped. 4 shows
FPV on robot 2, and F3 looks through robot 3's camera.

`controller_robots` does this. It finds each controller by its Bluetooth
address and reads its `/dev/input/jsN` itself, publishing `/robot_N/joy`
as `joy_node` would: `joy_node` picks a controller by index or by name, and
every DualSense has the same name.

Virtual pads are left out while a real controller is connected or has
driven: Sunshine makes an X-Box pad for a Moonlight stream, and Steam Input
a copy of a controller already there, so each would otherwise spawn a robot
nobody holds. One drives while it is all there is, and a real controller
connecting later takes over its robot. A pad counts as virtual when the
kernel has it under `/sys/devices/virtual` with no Bluetooth address, so
Bluetooth LE controllers, which come through `uhid` too, still count.

This is not in the editor's Run it command, which stays as the real arena
runs. There `robot_control.launch.py` binds each controller to the robot
`controllers.yaml` gives it, on the same `/robot_N/joy` and
`/robot_N/cmd_vel` topics, and hotplug would bind them in the order they
connect instead.

Checked headless on 2026-10-16 on a layout with ramps stacked on a 1"
platform up to a 2" one: a DualSense was bound to robot 2 with its motion
sensor; a virtual gamepad made through `/dev/uinput` spawned robot 3, drove
it about 2 m on R2, dropped, and drove robot 3 again on coming back; robot
1 came down the ramps to robot 2 on the floor and stopped 0.25 m away, and
after robot 2 was moved onto the 2" platform, climbed both levels (height
0.071 m) and stopped 0.23 m from it. F and 3 put the view on robots 2 and
3, and Ctrl-C stopped every process, robot 3's included.

Before it sets off, the hunter stows its arm: shoulder straight up, elbow
folded back over the chassis. It sends `+` and `X` on
`/robot_N/arm_command` one a tick, as the BLE bridge takes them, and holds
still until the arm is up. In the sim it sees that on
`/robot_N/joint_states`, in about 2 s; on a real robot it sends all 72
steps, which take 3.6 s and leave both servos at 180 degrees from wherever
they were. Stowed, the arm cannot catch on a wall as the hunter turns, and
its footprint is just its body, 10 by 7 cm with the wheels. It stows it
again whenever it starts tracking after losing a pose, in case the robot
was restarted, and `stow_arm:=false` leaves the arm alone.

It always heads for the target. When the way is clear it drives straight
at it, aiming where the target will be by the time it gets there. That
comes from the target's speed and turn rate, so a robot driving an arc is
led round the arc, and never more than `lead_time` (0.6 s) ahead, so it is
not sent off after a guess. Once it has caught up it adds the target's
speed to its own so it keeps pace, fading that out over the last 2 cm. It
stops with its front `clearance` (3 cm) short of the target: 0.13 m between
centres from behind or beside it, 0.23 m head on to an arm that is out. In
the sim it reads where the target's arm is from its `joint_states`; on the
real arena it takes the arm to be out. If the target comes closer it holds
its ground facing it, rather than backing off, and when the target circles
it, it drives after it rather than turning on the spot. If either pose is
older than 0.5 s, it stops.

The hunter knows the arena's terrain and plans its way to the target
(`terrain.py`). At start up it builds the arena from the same layout as the
sim (see Arena layouts below; `layout:=cad` reads
`models/robogame_arena_cad/model.sdf` instead) and turns its collision
geometry into a height map on a half inch grid. On a layout every module
edge is on that grid, so the middle of every gap between two modules is a
cell centre. The steps, walls, platform and ramp sides and the edge of the
floor, are found on a raster four times finer, so each cell knows how far
it is from the nearest wall and the nearest drop to a couple of millimetres.

- Where it has 6.3 cm to any wall, it can turn on the spot.
- Where it has less, down to 4.5 cm either side, it can still drive
  through lined up with the gap, along the grid or a diagonal, if its body
  fits that way round. A 4.5 inch gap at the end of a wall, or the 4 inch
  strip between a wall and the edge of the floor, is a way through, where
  before only gaps it could turn in were.
- A drop, the edge of the floor or of the platform it is on, only has to
  stay clear of its wheels, so the ends of its body may hang over it.
- On and within 7 cm of sloped ground it may only drive within 15 degrees
  of the slope, so it lines up before a ramp and goes straight up or down
  it, at least `ramp_speed` (0.15 m/s) either way so it does not stall on
  the steep top. Each ramp near it counts on its own: on a one module
  landing where one ramp comes up from one side and the next goes on up
  from another, as on the tower in test4, it drives along either, and once
  its body is clear of a ramp it may drive across the ramp's top edge. The
  ramps round it used to be averaged into one diagonal, which left such a
  landing, and so everything above it, out of reach.

Whenever it cannot drive straight at the target, it plans with A* over the
cells and, in the narrow ones, the way it faces. A plan can step sideways
into a lane beside it, a cell over in six along. It then pulls the path
straight, but never closer to a wall than the plan went, so a shortcut does
not shave a corner. It curves round corners in the open, no faster than
it can turn onto the curve, so it does not swing wide into a wall beside
it. At a corner where the next leg goes into a gap too narrow to turn in,
or up or down a ramp, it stops, turns to line up, and drives in, as a
player would. It stops back along the line of that leg, up to 5 cm short
of the corner, where it has the most room to turn, since the corner is
often right at the edge of where it can: stopping a centimetre wide of it
there would leave it unable to turn. Lined up in such a gap it plans as
lined up the way it faces, from the gap's cell beside it when a few
millimetres off a diagonal put it in one only lined up along the grid, and
keeps going the way it is unless backing out is 20 cm shorter, rather than
switching back and forth. It backs out when the target goes the other way,
heads for a point 12 cm on
along the gap so it eases back to the middle, and straightens up only
gently as it goes. Where it would creep a wheel over an edge doing that,
it turns where it stands instead, and where only a drop, not a wall,
stops it turning, as in a corner of the floor, it pivots slowly. It slows to `narrow_speed`
(0.15 m/s) in narrow gaps and a little near walls. If it has been pressed
against something for a second, or got nowhere for three, it backs off or
pulls forward for half a second, the other way the next time, but never a
way that would leave it over an edge. A target it
cannot get to, on a platform no ramp leads to, it waits for at the nearest
place it can turn. Where the way to the target goes round, it is as far
away as the way round, so a hunter on a ramp right under a target on the
platform above keeps climbing rather than stopping there as if with it. A
step takes about 3 ms, 25 ms at worst on a maze.

Try it without Gazebo:

```
python3 -m robogame_sim.chase_sim mazetest default --seeds 4
```

`chase_sim` runs `Chaser`, the steering `chase_node` uses, in a 2D stand-in
for the sim: both robots accelerate like the light robots they are, pose
noise and a tick of latency included, and a move is refused when the body
would hit a wall or a step or put a wheel over a drop. The target drives
itself still, wandering the arena, orbiting a spot, circling the hunter,
running from it or like a person on a joystick, and each run reports how
soon the hunter reached it, how much of the time after it stayed within
6 cm, and the longest it got nowhere while not with the target.

Checked on 2026-10-17. In `chase_sim`, 6 runs of 40 s for each kind of
target on each of seven layouts (the default, a flat floor, a maze and four
layouts with ramps, towers and walls), against the chase as it was before:

| | Before | Now |
| --- | --- | --- |
| Runs the target was reached in, six layouts | 155 of 216 | 216 of 216 |
| With the seventh | | 252 of 252 |
| Longest the hunter got nowhere | 40 s, the whole run | 3.5 s |
| Moves refused, body or arm against something | thousands a layout | tens |
| Planning a step, average | 2 to 14 ms | 1 to 4 ms |

Before, the hunter's arm caught on walls as it turned and it never found
the 4.5 inch gaps it could only drive through lined up, so in the maze and
walled layouts it mostly sat stuck from the start.

Headless in Gazebo, robot 2 driven by a script, robot 1 hunting it. With
it is the time after first reaching it that its front was within 6 cm:

| Layout | Robot 2 | Reached | With it | Longest stall |
| --- | --- | --- | --- | --- |
| default | circling robot 1 | 2.3 s | 100% | 0.2 s |
| default | joystick-like | 2.4 s | 100% | 0.2 s |
| maze | to the far room, up the 4 inch strip by the floor's edge | 9.8 s | 100% | 1.5 s |
| maze | joystick-like, two runs | 9.4, 11.1 s | 100, 99% | 1.7 s |
| maze | wandering | 8.2 s | 100% | 1.6 s |
| maze | running away | 10.0 s | 100% | 1.6 s |
| ramps and walls | wandering | 3.6 s | 85% | 1.6 s |
| ramps and walls | joystick-like | 3.6 s | 98% | 0.8 s |
| ramps, towers | wandering | 3.4 s | 100% | 0.9 s |

In every run the arm was up within 2 s and stayed there (shoulder +1.57,
elbow +1.48 rad). The maze runs also found three ways the hunter could end
up over the floor's edge in a corner, which the 2D stand-in had not shown,
and each is now guarded against.

Checked again on 2026-10-18 on test4: a tower in the middle with a ramp
spiralling up its outside, one module wide with a drop on the outer side,
four legs turning on one module landings, up to its 8 inch top; round it a
box of walls with an 8 inch gap in the middle of each side and, at each
corner, a diagonal slot between the walls' ends that the robot only gets
through lined up with it. Headless in Gazebo at 2 ms steps, four runs at a
time, robot 2 put somewhere by the script or driven like a player, robot 1
hunting it. Lane is the furthest the hunter's centre got from the middle
of a spiral leg, which leaves 1.6 cm either side of its wheels:

| Robot 2 | Reached | Through the slot | Lane | Freed itself |
| --- | --- | --- | --- | --- |
| just outside each corner slot, hunter inside the box, four runs | 3.0 to 4.2 s | 0 to 5 degrees off its line, 0.1 to 0.9 cm off its middle | | 0 |
| inside past a corner slot, hunter outside it, two runs | 2.5, 3.8 s | 2 and 5 degrees, 0.5 cm | | 0 |
| driving out through a corner slot, hunter after it | 5.1 s | 5 degrees, 0.7 cm | | once |
| on the tower top, hunter on the floor, four starts | 18.8 to 25.9 s | | 1.2 cm | 0 |
| driving up the spiral, hunter after it | to the top | | 1.1 cm | once |
| on the top, hunter knocked 1.2 cm off line and turned 11 degrees toward the drop, twice on the way up, two runs | 21.9, 24.0 s | | 1.5 cm | at most once |

It never fell, and the heading error after a knock was gone within half
a second. Run them again with

```
scripts/gazebo_chase_suite.sh /tmp/chase_runs              # all 14
scripts/gazebo_chase_suite.sh /tmp/chase_runs tower_nudge  # just one
```

which takes a few minutes for all of them, four at a time
(`gazebo_chase_test.py` has the scenarios, `gazebo_chase_score.py` scores
a run and with `--plot` draws it). Before, the top of the tower was out of reach: the hunter waited
at the foot of the spiral. Through the slots, it would stop on a corner at
the very edge of where it could turn, end up a centimetre off it and press
against the end of a wall, and on its way to the foot of the spiral it
swung wide into a strip by the wall too narrow to turn in.

In `chase_sim`, 18 runs of 40 s on each layout, against the chase before
these changes: every run on the default, flat, maze and test1 and test4
layouts still reaches the target; on test2 and test3, 16 of 18, as a target
running away can now loop round the arena over the corner platforms, which
both robots can use now, and in two runs stayed ahead for the whole 40 s.
On test4 moves refused fell from 54 to 12.

## Robot console

A page for every robot in the arena: which controller drives which
robot, which robots drive themselves, and where those go.

```
ros2 run robogame_sim robot_console
```

It opens http://localhost:8767 in the browser (only this computer can
reach it; `--port` and `--no-browser` change that). Start it beside a sim,
or on the real arena: it finds robots by their `/model/robot_N/odometry`,
which the sim and the arena camera both publish, and leaves a robot off
the page 10 s after its odometry goes.

The arena is drawn as the editor draws it, with every robot on it, in its
colour and the way it faces. It is the arena in use:

- In the sim, the layout `sim.launch.py` built the world from. The launch
  publishes it, latched, on `/arena/layout` (`active_arena`).
- With nothing published there, as on the real arena, the layout last
  saved in the arena editor, since that is where the modules on the table
  are mapped. To pin one, run
  `ros2 run robogame_sim active_arena --ros-args -p layout:=NAME`.
- The Arena list on the page picks any saved layout, or `cad`, over both.

The planner uses the same arena, and a new one is loaded as soon as it
changes, a layout saved again included.

Controllers: each one `controller_robots` has is listed with the robot it
drives, and picking another robot moves it there, its racing controls
and wheel steering with it. A controller moved onto a robot another one
drives swaps robots with it; No robot takes it off every robot. This needs
`controller_robots`, so in the sim launch with `controller:=hotplug`. On
the real arena `robot_control.launch.py` binds controllers from
`controllers.yaml`, and the page says no `controller_robots` is running.

Pick a robot in the list, or with keys 1 to 9, and make it Autonomous.
The console then drives it on `/robot_N/cmd_vel` with `Chaser`, the
steering `chase_node` uses, so it plans round platforms, lines up for
narrow gaps and climbs ramps the same way. It first stows its arm, as
`chase_node` does. Its controller is taken off it while it is autonomous,
and given back when it is made Manual again. If anything else publishes
its `cmd_vel`, such as `chase_node` or the real arena's
`controller_input`, the page names it, since the two would fight. Its
planned path is drawn on the arena in its colour, bold for the robot
picked and faint for the others.

It has two modes:

- Commands, the default. Go to a robot, picked from a list, stops just
  short of it as the chase does. Go to a spot takes x and y in metres from
  the middle of the arena, +x right and +y up, or a click on the arena.
  Clicking a robot goes to it. Go replaces what it is doing; Queue, or a
  Shift-click, adds the command to the end of its queue, and it does them
  one after another. Queued spots are numbered on the arena. A spot is
  reached within 3 cm. A spot it cannot reach, on a platform no ramp leads
  to, it goes as close to as it can and moves on. Stop and clear, or Esc,
  stops it.
- Chase follows the robot picked, or clicked, for as long as it is left
  on, as `chase_node` does.

When a robot's pose is older than 0.5 s it is stopped, and it stows its
arm again when the pose comes back. Ctrl-C in the console's terminal stops
every robot it drives.

Checked on 2026-10-18. 17 tests: `test_autopilot.py` runs the autopilot
in `chase_sim`'s 2D stand-in (up the default platform's ramp to a spot on
top, a spot then a robot from its queue, a spot it cannot reach, a chase
after a robot that moves, the arm stowed first), and
`test_robot_console.py` runs the console on a ROS domain of its own
against a stand-in sim and `controller_robots`, through the page's API.
Then headless in Gazebo on test4 with three robots: robot 1, sent to the
top of the tower by the page, climbed the spiral to it (z 0.223 m) in
about 45 s and stopped within 3 cm of the spot; robot 2 in chase mode
went after robot 3 and held beside it; clicks on the arena, in headless
Chromium, queued spots and robots where they were clicked to a millimetre,
and the Chase and Manual buttons switched mode. With `controller_robots`
and a DualSense, the controller moved from robot 3 to robot 1, came off
robot 1 when it was made autonomous and back when it was made manual, and
went to no robot and back to 3. Stopping the sim stopped robot 2 and
switched the arena to the layout last saved in the editor. The editor,
which now loads its drawing code from `arena_draw.js` shared with the
console, was checked in headless Chromium to draw test4 as before.

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

Every module pegs into the same 2 inch grid of holes, four to a tile and
evenly spaced across the joins between tiles, so a module can go half a
square along and stand across two tiles. A wall or thwomp takes one row of
two holes, so two fit on a tile side by side, or one each way.

Any module can stand on a platform of either height, and platforms stack,
so a ramp on a 1" platform climbs onto a 2" platform or onto two 1"
platforms stacked. A module can stand across two platforms if they are as
high.

Map how they are set up on the table in the layout editor:

```
ros2 run robogame_sim arena_editor
```

It opens http://localhost:8766 in the browser (only this computer can
reach it; `--port` and `--no-browser` change that). Pick a module (keys 1
to 5) and click or drag on the grid, which is seen from above with +x to
the right and +y up, each square drawn with its tile's four peg holes.
The module goes where the pointer is, to the nearest half square; turn off
Half squares (H) to keep to whole squares. A wall or thwomp goes on the
line of peg holes under the pointer, along the way it faces.
Click a ramp, wall or thwomp with its own tool to turn it a quarter turn,
or press R over it. A ramp's arrow points uphill.
Shift-click, or turn on Stack (S), to put the tool on top of platforms
instead of replacing them; a stack shows how many inches up its top is in
its corner. A plain click replaces whatever is in the way, at the height
of the top of what is there. Right-click or the Floor tool (6) takes the
top module under the pointer off, along with anything standing on it, and
Ctrl+Z undoes. As
you edit it checks the layout and outlines in red any ramp whose low end
runs into the edge, a module, a wall facing it, the side of a ramp or a
drop, any ramp whose high end is not a platform (or the foot of another
ramp) exactly an inch above where it starts, and any platform robots are
meant to reach (a 1" one, or one a ramp leads to) that no robot can get
onto from the floor. Ramps chain, so a ramp on a platform can carry on
from the top of one on the floor, and a chain can turn on a one module
landing. The checks go by which module edges meet at the same height;
chase_node's planner is more cautious: the robot only turns where it has
6.3 cm to any wall, so it may not get round a one module landing against
a wall that a robot driven by hand gets over. Name the layout and Save,
and it is written to
`src/robogame_sim/layouts/NAME.json`, one module per line so it diffs
well. `http://localhost:8766/#NAME` opens a saved one directly.

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
export as it is, MAIN ASSEMBLY's modules where it puts them. A module on a platform has
`"level": 1` in the file (2 on top of that, and so on), and is left
without one on the floor. `rotation`
in the file is degrees anticlockwise from above: a ramp at 0 rises toward
+y, 90 toward -x, 180 toward -y, 270 toward +x, and a wall or thwomp at
those stands on the +y, -x, -y or +x side of its square, an inch from
the middle. `col` and `row` go in halves: `"col": 5.5` is across the tiles
at cols 5 and 6.

Robots spawn on a circle round the middle, and one whose spot is on a
ramp or wedged against a module (closer than 8 cm, as at the corners of
the default platform) is moved to the nearest open, level ground at the
same height, and the launch prints where. A robot that would face a module
less than 25 cm away is turned to face open ground, as its arm, reaching
15 cm past its centre, would pin it there.

Only one sim can run at a time in a Gazebo partition, since they share the
world's name. If one is already running the launch stops straight away and
says so; before, the second one looked frozen, as its windows and nodes
attached to the first. Stop the first, or add `partition:=NAME` to run the
second beside it. That separates the two in Gazebo only: both still use
`/robot_N/...` in ROS, so the two would drive each other's robots. Give the
second its own ROS domain as well, `ROS_DOMAIN_ID=42 ros2 launch ...`, and
the same in any terminal that talks to it.

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
cd src/robogame_sim && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test
```

With ROS sourced, its pytest plugins stop pytest finding the tests, hence
`PYTEST_DISABLE_PLUGIN_AUTOLOAD`. These check the firmware maths against
motors.cpp and arm.cpp, the racing controls and steering wheel maths, that
the robot model is valid XML, the chase and path planning, the chase
against moving targets in `chase_sim` (`test_chase_dynamic.py`), the
arena layouts and their editor, and the robot console
(`test_autopilot.py`, and `test_robot_console.py`, which needs ROS
sourced). Print a robot's SDF with `ros2 run robogame_sim
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
