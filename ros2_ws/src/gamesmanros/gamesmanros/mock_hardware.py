#!/usr/bin/env python3
"""
Mock arm + gripper action servers.

Stands in for arm_controller and gripper_controller so the rest of the stack can
be exercised with no robot, no MoveIt configuration and no mycobot_interfaces.
Everything upstream is real: UWAPI, the Type classes, the coordinate pipeline and
the full ROS 2 action plumbing. Only the final hop to hardware is simulated.

    ros2 run gamesmanros mock_hardware

Parameters
  arm_delay          seconds to simulate a planned motion      (default 0.15)
  gripper_delay      seconds to simulate an actuation          (default 0.10)
  workspace_radius   metres; goals beyond this are rejected to
                     mimic an IK failure. 0 disables.          (default 0.0)
  verbose            log every goal                            (default True)

Deliberately imports nothing from moveit or mycobot_interfaces.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from gamesmanros_interfaces.action import MoveArm, MoveGripper


class MockHardware(Node):
    def __init__(self):
        super().__init__('mock_hardware')

        self.declare_parameter('arm_delay', 0.15)
        self.declare_parameter('gripper_delay', 0.10)
        self.declare_parameter('workspace_radius', 0.0)
        self.declare_parameter('verbose', True)

        self._arm_delay = self.get_parameter('arm_delay').value
        self._grip_delay = self.get_parameter('gripper_delay').value
        self._radius = self.get_parameter('workspace_radius').value
        self._verbose = self.get_parameter('verbose').value

        cb = ReentrantCallbackGroup()

        self._arm_srv = ActionServer(
            self, MoveArm, 'move_arm',
            execute_callback=self._do_arm,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.REJECT,
            callback_group=cb)

        self._grip_srv = ActionServer(
            self, MoveGripper, 'move_gripper',
            execute_callback=self._do_gripper,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.REJECT,
            callback_group=cb)

        self.arm_goals = 0
        self.grip_goals = 0
        self.failures = 0

        self.get_logger().warn('=' * 58)
        self.get_logger().warn('  MOCK HARDWARE — no robot is connected')
        self.get_logger().warn('  serving /move_arm and /move_gripper')
        if self._radius > 0:
            self.get_logger().warn(f'  simulating workspace limit r={self._radius:.3f} m')
        self.get_logger().warn('=' * 58)

    # ------------------------------------------------------------------

    def _do_arm(self, goal_handle):
        req = goal_handle.request
        self.arm_goals += 1
        n = self.arm_goals

        fb = MoveArm.Feedback()
        fb.status = f'Planning to ({req.x:.3f}, {req.y:.3f}, {req.z:.3f})'
        goal_handle.publish_feedback(fb)

        result = MoveArm.Result()
        reach = math.hypot(req.x, req.y)

        if self._radius > 0 and reach > self._radius:
            self.failures += 1
            self.get_logger().error(
                f'[arm #{n}] UNREACHABLE  ({req.x:+.3f}, {req.y:.3f}, {req.z:.3f})  '
                f'reach={reach:.3f} > {self._radius:.3f}')
            result.success = False
            result.message = 'Simulated IK failure'
            goal_handle.abort()
            return result

        time.sleep(self._arm_delay)

        fb.status = 'Executing trajectory'
        goal_handle.publish_feedback(fb)

        if self._verbose:
            self.get_logger().info(
                f'[arm #{n}] moved to ({req.x:+.3f}, {req.y:.3f}, {req.z:.3f})  '
                f'reach={reach:.3f} m')

        result.success = True
        result.message = 'OK (mock)'
        goal_handle.succeed()
        return result

    def _do_gripper(self, goal_handle):
        self.grip_goals += 1
        n = self.grip_goals
        opening = goal_handle.request.open

        fb = MoveGripper.Feedback()
        fb.status = 'opening' if opening else 'closing'
        goal_handle.publish_feedback(fb)

        time.sleep(self._grip_delay)

        if self._verbose:
            self.get_logger().info(
                f'[grip #{n}] {"OPEN " if opening else "CLOSE"}')

        result = MoveGripper.Result()
        result.success = True
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = MockHardware()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'totals — arm: {node.arm_goals}, gripper: {node.grip_goals}, '
            f'failures: {node.failures}')
        node.destroy_node()
        rclpy.shutdown()
