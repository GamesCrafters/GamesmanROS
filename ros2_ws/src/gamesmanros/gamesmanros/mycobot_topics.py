#!/usr/bin/env python3
"""
MyCobot serial driver — a port of the ROS 1 mycobot_topics.py.

Runs on whatever machine the arm is plugged into. It is the only node that talks
to the arm:

    /mycobot/angles_real   (out)  joint angles in degrees, polled from the arm
    /mycobot/angles_goal   (in)   move the arm to these angles (degrees) at `speed`
    /mycobot/gripper_status(in)   status=true opens the gripper, false closes it

Parameters
  port             serial device                       (default /dev/ttyUSB0;
                                                        Pi-mounted arm: /dev/ttyAMA0)
  baud             baud rate                           (default 115200; Pi: 1000000)
  publish_period   seconds between angle polls         (default 0.25, as in ROS 1)
  gripper_speed    speed passed to set_gripper_state   (default 80, as in ROS 1)

Needs `pip install pymycobot` (imported lazily so this module imports without it).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from mycobot_interfaces.msg import (
    MycobotAngles, MycobotSetAngles, MycobotGripperStatus)

_GRIPPER_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


def gripper_flag(open_: bool) -> int:
    """pymycobot's set_gripper_state flag: 0 opens, 1 closes (as in ROS 1)."""
    return 0 if open_ else 1


def clamp_speed(speed: int) -> int:
    return max(1, min(100, int(speed)))


class MycobotTopics(Node):
    def __init__(self):
        super().__init__('mycobot_topics')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('publish_period', 0.25)
        self.declare_parameter('gripper_speed', 80)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self._gripper_speed = int(self.get_parameter('gripper_speed').value)
        try:
            from pymycobot.mycobot import MyCobot
        except ImportError:
            from pymycobot import MyCobot
        self.get_logger().info(f'connecting to the arm on {port} @ {baud}')
        self._mc = MyCobot(port, baud)

        self._pub = self.create_publisher(MycobotAngles, 'mycobot/angles_real', 5)
        self.create_subscription(
            MycobotSetAngles, 'mycobot/angles_goal', self._on_goal, 10)
        self.create_subscription(
            MycobotGripperStatus, 'mycobot/gripper_status', self._on_gripper,
            _GRIPPER_QOS)
        self.create_timer(float(self.get_parameter('publish_period').value),
                          self._publish_angles)
        self.get_logger().info('MycobotTopics ready')

    def _publish_angles(self):
        angles = self._mc.get_angles()
        if not angles or len(angles) < 6:
            return
        msg = MycobotAngles()
        (msg.joint_1, msg.joint_2, msg.joint_3,
         msg.joint_4, msg.joint_5, msg.joint_6) = (float(a) for a in angles[:6])
        self._pub.publish(msg)

    def _on_goal(self, msg: MycobotSetAngles):
        angles = [msg.joint_1, msg.joint_2, msg.joint_3,
                  msg.joint_4, msg.joint_5, msg.joint_6]
        self._mc.send_angles(angles, clamp_speed(msg.speed))

    def _on_gripper(self, msg: MycobotGripperStatus):
        self._mc.set_gripper_state(gripper_flag(msg.status), self._gripper_speed)


def main():
    rclpy.init()
    node = MycobotTopics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
