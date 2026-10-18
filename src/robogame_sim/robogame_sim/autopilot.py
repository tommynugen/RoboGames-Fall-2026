"""Drives a robot the robot console has made autonomous, kept free of ROS
so it can be tested.

An Autopilot has two modes. In command mode it works through a queue of
commands one after another: go to a spot on the arena, or go to a robot
and stop beside it. In chase mode it follows a robot for as long as it is
left on, as chase_node does. Both steer with Chaser, so the robot plans
round platforms, lines up for narrow gaps and climbs ramps the same way
the chase does, and both stow the arm first.
"""

import math

from robogame_sim.chase import STOW_ARM, STOWED, Chaser

COMMAND = "command"
CHASE = "chase"
MODES = (COMMAND, CHASE)

# A spot counts as reached with the robot's centre this close to it.
ARRIVE = 0.03


def point(x, y):
    """A command to drive the robot's centre onto x, y, in metres."""
    x, y = float(x), float(y)
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("a spot needs numbers for x and y")
    return {"kind": "point", "x": round(x, 4), "y": round(y, 4)}


def robot(robot_id):
    """A command to drive up to a robot and stop beside it."""
    return {"kind": "robot", "robot": int(robot_id)}


def describe(goal):
    if goal["kind"] == "point":
        return f"to {goal['x']:.2f}, {goal['y']:.2f}"
    return f"to robot {goal['robot']}"


class Autopilot:
    """Feed it the robot's pose each tick with update, and the poses of the
    robots it can see; it answers with a twist and, while it stows the arm,
    an arm command."""

    def __init__(self, robot_id, terrain=None, **chaser_options):
        self.robot_id = robot_id
        self.terrain = terrain
        self.chaser_options = chaser_options
        self.mode = COMMAND
        # Commands still to do, the one being done first.
        self.queue = []
        self.chase_target = None
        self.chaser = None
        # What the chaser was made for, so a new one is made when it changes.
        self.goal = None
        self.arm_queue = ""
        self.arm = None
        self.status = "idle"
        self.last = ""
        self.path = None

    # -- what to do --------------------------------------------------------

    def set_terrain(self, terrain):
        self.terrain = terrain
        self.goal = None

    def set_mode(self, mode):
        if mode not in MODES:
            raise ValueError(f"mode must be one of {', '.join(MODES)}")
        self.mode = mode

    def go(self, command, append=False):
        """Does command next, after the ones queued when append."""
        if append:
            self.queue.append(command)
        else:
            self.queue = [command]

    def remove(self, index):
        del self.queue[index]

    def stop(self):
        self.queue = []
        self.chase_target = None
        self.last = "stopped"

    def chase(self, robot_id):
        if robot_id is not None and int(robot_id) == self.robot_id:
            raise ValueError("a robot cannot chase itself")
        self.chase_target = None if robot_id is None else int(robot_id)

    def current(self):
        if self.mode == CHASE:
            if self.chase_target is None:
                return None
            return {"kind": "chase", "robot": self.chase_target}
        return self.queue[0] if self.queue else None

    # -- the arm -----------------------------------------------------------

    def stow(self):
        """Stows the arm before driving, one step a tick as the BLE bridge
        takes them. Called on taking the robot over, and on getting its
        pose back, in case it was restarted."""
        self.arm_queue = STOW_ARM

    def set_arm(self, shoulder, elbow):
        self.arm = (shoulder, elbow)

    # -- driving -----------------------------------------------------------

    def pose_lost(self):
        self.status = "no pose, stopped"
        self.path = None

    def finish(self, message):
        self.queue.pop(0)
        self.last = message
        self.goal = None
        self.path = None

    def update(self, now, x, y, yaw, robots):
        """(linear, angular, arm command or None) for the robot at x, y,
        yaw. now is the time of that pose. robots maps the other robots
        seen to (x, y, yaw, stamp, arm), arm being (shoulder, elbow) or
        None."""
        if self.arm_queue:
            step, self.arm_queue = self.arm_queue[0], self.arm_queue[1:]
            if self.arm is not None and min(self.arm) >= STOWED:
                self.arm_queue = ""
            self.status = "stowing the arm"
            self.path = None
            return 0.0, 0.0, step

        goal = self.current()
        if goal != self.goal:
            self.goal = goal
            self.chaser = goal and Chaser(self.terrain,
                                          **self.chaser_options)
            if goal and goal["kind"] == "point":
                self.chaser.point = True

        if goal is None:
            self.status = ("pick a robot to chase" if self.mode == CHASE
                           else "idle")
            self.path = None
            return 0.0, 0.0, None

        chaser = self.chaser
        if goal["kind"] == "point":
            gx, gy = goal["x"], goal["y"]
            if math.dist((x, y), (gx, gy)) < ARRIVE:
                self.finish(f"reached {gx:.2f}, {gy:.2f}")
                self.status = "idle"
                return 0.0, 0.0, None
            chaser.track(gx, gy, now)
            name = "the spot"
        else:
            seen = robots.get(goal["robot"])
            if seen is None:
                self.status = f"waiting to see robot {goal['robot']}"
                self.path = None
                return 0.0, 0.0, None
            tx, ty, tyaw, stamp, arm = seen
            chaser.track(tx, ty, stamp, tyaw)
            if arm is not None:
                chaser.set_target_arm(*arm)
            gx, gy = tx, ty
            name = f"robot {goal['robot']}"

        linear, angular = chaser.update(now, x, y, yaw)
        self.path = ([tuple(map(float, p)) for p in chaser.path]
                     if chaser.path is not None else [(x, y), (gx, gy)])
        status = chaser.status.replace("the target", name)
        if status == "path":
            status = f"on its path to {name}"

        if goal["kind"] == "point" and chaser.waiting:
            self.finish(f"could not reach {gx:.2f}, {gy:.2f}, stopped as "
                        f"close as it gets")
            self.status = "idle"
            return 0.0, 0.0, None

        if goal["kind"] == "robot" and chaser.engaged:
            self.finish(f"reached robot {goal['robot']}")
            self.status = "idle"
            return 0.0, 0.0, None

        what = (f"chasing robot {goal['robot']}" if goal["kind"] == "chase"
                else describe(goal))
        self.status = f"{what}: {status}"
        return linear, angular, None

    def state(self):
        return {
            "mode": self.mode,
            "queue": list(self.queue),
            "chase_target": self.chase_target,
            "status": self.status,
            "last": self.last,
            "path": self.path,
        }
