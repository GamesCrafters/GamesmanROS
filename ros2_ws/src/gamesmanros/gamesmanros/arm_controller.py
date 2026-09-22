#!/usr/bin/env python3
"""
Arm motion action server using MoveIt 2 (moveit_py).

Key fix over ROS 1: uses OMPL/RRTConnect (joint-space) instead of PILZ LIN
(Cartesian-space), so planning succeeds even when no straight-line IK path
exists between poses.

Fixed end-effector orientation: gripper pointing down, rotated 45° about Z,
with a -11.25° tilt about X to compensate for the MyCobot wrist angle.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler, quaternion_multiply

from moveit.planning import MoveItPy

from gamesmanros_interfaces.action import MoveArm

BASE_FRAME = 'base_link'
PLANNING_GROUP = 'arm_group'
EEF_LINK = 'joint6_flange'

# Compose fixed end-effector orientation (same angles as ROS 1 low_level_controller):
#   q1: 180° about X  (point gripper down)
#   q2: 45° about Z   (align with board approach angle)
#   q3: -11.25° about X (slight wrist correction)
_q1 = quaternion_from_euler(math.pi, 0, 0)
_q2 = quaternion_from_euler(0, 0, math.pi / 4)
_q3 = quaternion_from_euler(-math.pi / 16, 0, 0)
_EEF_ORIENTATION = quaternion_multiply(quaternion_multiply(_q1, _q2), _q3)  # [x,y,z,w]


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self._moveit = MoveItPy(node_name='arm_controller_moveit')
        self._arm = self._moveit.get_planning_component(PLANNING_GROUP)

        self._action_server = ActionServer(
            self,
            MoveArm,
            'move_arm',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.REJECT,
        )
        self.get_logger().info('ArmController ready')

    def _build_pose(self, x: float, y: float, z: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        q = _EEF_ORIENTATION
        pose.pose.orientation.x = round(q[0], 6)
        pose.pose.orientation.y = round(q[1], 6)
        pose.pose.orientation.z = round(q[2], 6)
        pose.pose.orientation.w = round(q[3], 6)
        return pose

    def _execute(self, goal_handle):
        req = goal_handle.request
        x, y, z = req.x, req.y, req.z

        feedback = MoveArm.Feedback()
        feedback.status = f'Planning to ({x:.3f}, {y:.3f}, {z:.3f})'
        goal_handle.publish_feedback(feedback)

        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(
            pose_stamped_msg=self._build_pose(x, y, z),
            pose_link=EEF_LINK,
        )

        plan_result = self._arm.plan()

        result = MoveArm.Result()
        if not plan_result:
            self.get_logger().warn(f'Planning failed for ({x:.3f}, {y:.3f}, {z:.3f})')
            result.success = False
            result.message = 'Planning failed'
            goal_handle.abort()
            return result

        feedback.status = 'Executing trajectory'
        goal_handle.publish_feedback(feedback)

        self._moveit.execute(plan_result.trajectory, blocking=True, controllers=[])

        result.success = True
        result.message = 'OK'
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = ArmController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
