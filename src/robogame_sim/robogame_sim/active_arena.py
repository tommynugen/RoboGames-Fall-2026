"""Says which arena layout is in use, for the robot console.

    ros2 run robogame_sim active_arena --ros-args -p layout:=NAME

Publishes it once, latched, on /arena/layout as JSON: {"name": ...,
"source": ..., "layout": {...}}, layout being the layout file's contents,
or null for cad, the Onshape export as it is. sim.launch.py starts one for
the layout it built the world from, with source gazebo. On the real arena
run it with the layout set up on the table; without one the console takes
the layout last saved in the arena editor.
"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from robogame_sim import arena_layout

TOPIC = "/arena/layout"
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def message(name, source):
    """The JSON published for layout name, a name or path, or cad."""
    if name == "cad":
        layout = None
    else:
        layout = arena_layout.load(name)
        if name.endswith(".json"):
            name = name.rsplit("/", 1)[-1][:-5]

    return json.dumps({"name": name, "source": source, "layout": layout})


class ActiveArena(Node):

    def __init__(self):
        super().__init__("active_arena")
        self.declare_parameter("layout", "default")
        self.declare_parameter("source", "real")
        name = self.get_parameter("layout").value
        source = self.get_parameter("source").value

        self.publisher = self.create_publisher(String, TOPIC, LATCHED)
        self.publisher.publish(String(data=message(name, source)))
        self.get_logger().info(f"Arena in use: {name}, {source}")


def main(args=None):
    rclpy.init(args=args)
    node = ActiveArena()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
