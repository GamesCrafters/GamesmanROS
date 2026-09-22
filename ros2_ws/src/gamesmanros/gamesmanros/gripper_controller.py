#!/usr/bin/env python3
"""
Gripper action server.

Replaces the ROS 1 hack of spamming 7+ repeated publishes with sleep().
Publishes a single command with RELIABLE QoS and waits the hardware
actuation time (1.5 s) before reporting success.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from gamesmanros_interfaces.action import MoveGripper
from mycobot_interfaces.msg import MycobotGripperStatus

ACTUATION_TIME = 1.5  # seconds — empirical hardware settling time

_GRIPPER_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        self._pub = self.create_publisher(
            MycobotGripperStatus, '/mycobot/gripper_status', _GRIPPER_QOS)

        self._action_server = ActionServer(
            self,
            MoveGripper,
            'move_gripper',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.REJECT,
        )
        self.get_logger().info('GripperController ready')

    def _execute(self, goal_handle):
        open_gripper = goal_handle.request.open
        state_str = 'open' if open_gripper else 'close'

        feedback = MoveGripper.Feedback()
        feedback.status = f'Sending gripper {state_str} command'
        goal_handle.publish_feedback(feedback)

        msg = MycobotGripperStatus()
        msg.Status = 1 if open_gripper else 0
        self._pub.publish(msg)

        # Hardware needs time to actuate; no need to spam publishes
        # because RELIABLE QoS guarantees delivery.
        time.sleep(ACTUATION_TIME)

        goal_handle.succeed()
        result = MoveGripper.Result()
        result.success = True
        return result


def main():
    rclpy.init()
    node = GripperController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
