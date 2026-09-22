#!/usr/bin/env python3
"""
Bridges MyCobot joint angles (degrees) to /joint_states (radians) for MoveIt 2.
Runs on the Pi alongside the mycobot_ros2 driver.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from mycobot_interfaces.msg import MycobotAngles

JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')

        self._pub = self.create_publisher(JointState, '/joint_states', 10)
        self._sub = self.create_subscription(
            MycobotAngles, '/mycobot/angles_real', self._cb, 10)

        self._msg = JointState()
        self._msg.name = JOINT_NAMES
        self._msg.position = [0.0] * 6

    def _cb(self, msg: MycobotAngles):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._msg.position = [
            math.radians(msg.joint_1),
            math.radians(msg.joint_2),
            math.radians(msg.joint_3),
            math.radians(msg.joint_4),
            math.radians(msg.joint_5),
            math.radians(msg.joint_6),
        ]
        self._pub.publish(self._msg)


def main():
    rclpy.init()
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
