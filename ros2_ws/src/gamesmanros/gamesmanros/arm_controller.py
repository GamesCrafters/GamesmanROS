#!/usr/bin/env python3
"""
Arm motion action server using MoveIt 2 (moveit_py).

Planner strategy: PILZ LIN first, OMPL as a fallback.

The ROS 1 system used LIN alone. LIN produces straight-line Cartesian motion,
which is deliberately predictable above a dense board — the arm will not sweep
through neighbouring pieces on its way somewhere. That predictability is worth
keeping (see EECS-2025-128 section 3.3). Its weakness is that it needs a valid
IK solution at EVERY interpolated waypoint, so it fails outright when no
straight-line path exists between two poses.

OMPL/RRTConnect searches joint space instead, so it only needs IK at the two
endpoints — but its Cartesian path is unconstrained and may swing the elbow
across the board.

So: try LIN, and only fall back to OMPL when LIN cannot find a plan. Normal
moves stay predictable; the straight-line failures that motivated the ROS 2
rewrite stop being fatal. In ROS 1 this was impossible because a failed plan
returned False and was silently dropped — the caller never knew to retry.

Every goal logs which planner served it, so `planner_stats` tells you how often
LIN actually fails on the real board. If LIN turns out to be reliable the
fallback costs nothing; if it does not, that is the data needed to make OMPL
primary.

Requires BOTH planning pipelines enabled in the moveit_config:
    ompl  and  pilz_industrial_motion_planner

Fixed end-effector orientation: gripper pointing down, rotated 45° about Z,
with a -11.25° tilt about X to compensate for the MyCobot wrist angle.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler, quaternion_multiply

from moveit.planning import MoveItPy, PlanRequestParameters

from gamesmanros_interfaces.action import MoveArm

BASE_FRAME = 'g_base'  # root link of mycobot_280_moveit_config's URDF
LIN_PIPELINE  = 'pilz_industrial_motion_planner'
LIN_PLANNER   = 'LIN'
OMPL_PIPELINE = 'ompl'
OMPL_PLANNER  = 'RRTConnectkConfigDefault'
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

        self.declare_parameter('use_lin_first', True)
        self._use_lin_first = self.get_parameter('use_lin_first').value

        self._lin_params = self._make_params(LIN_PIPELINE, LIN_PLANNER)
        self._ompl_params = self._make_params(OMPL_PIPELINE, OMPL_PLANNER)
        if self._lin_params is None:
            self.get_logger().warn(
                f'{LIN_PIPELINE} pipeline unavailable — running OMPL only. '
                f'Enable it in the moveit_config to get predictable Cartesian motion.')
            self._use_lin_first = False

        # How often each planner actually served a goal.
        self.planner_stats = {'lin': 0, 'ompl_fallback': 0, 'failed': 0}

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

    def _make_params(self, pipeline: str, planner: str):
        """Build PlanRequestParameters, or None if that pipeline is not configured."""
        try:
            p = PlanRequestParameters(self._moveit, planner)
            p.planning_pipeline = pipeline
            p.planner_id = planner
            return p
        except Exception as e:
            self.get_logger().warn(f'pipeline {pipeline!r} unavailable: {e}')
            return None

    def _plan(self, x, y, z):
        """Try LIN, fall back to OMPL. Returns (plan_result, planner_name)."""
        pose = self._build_pose(x, y, z)

        if self._use_lin_first and self._lin_params is not None:
            self._arm.set_start_state_to_current_state()
            self._arm.set_goal_state(pose_stamped_msg=pose, pose_link=EEF_LINK)
            plan = self._arm.plan(single_plan_parameters=self._lin_params)
            if plan:
                self.planner_stats['lin'] += 1
                return plan, 'LIN'
            self.get_logger().warn(
                f'LIN found no straight-line path to '
                f'({x:.3f}, {y:.3f}, {z:.3f}) — falling back to OMPL')

        # Re-seed the start state; the failed attempt leaves it stale.
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped_msg=pose, pose_link=EEF_LINK)
        plan = (self._arm.plan(single_plan_parameters=self._ompl_params)
                if self._ompl_params is not None else self._arm.plan())
        if plan:
            key = 'ompl_fallback' if self._use_lin_first else 'lin'
            self.planner_stats[key] += 1
            return plan, 'OMPL'
        return None, None

    def _execute(self, goal_handle):
        req = goal_handle.request
        x, y, z = req.x, req.y, req.z

        feedback = MoveArm.Feedback()
        feedback.status = f'Planning to ({x:.3f}, {y:.3f}, {z:.3f})'
        goal_handle.publish_feedback(feedback)

        plan_result, planner = self._plan(x, y, z)

        result = MoveArm.Result()
        if not plan_result:
            self.planner_stats['failed'] += 1
            self.get_logger().warn(
                f'Planning failed for ({x:.3f}, {y:.3f}, {z:.3f}) — '
                f'both LIN and OMPL' if self._use_lin_first else
                f'Planning failed for ({x:.3f}, {y:.3f}, {z:.3f})')
            result.success = False
            result.message = 'Planning failed'
            goal_handle.abort()
            return result

        feedback.status = 'Executing trajectory'
        goal_handle.publish_feedback(feedback)

        self._moveit.execute(plan_result.trajectory, blocking=True, controllers=[])

        result.success = True
        result.message = f'OK ({planner})'
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        st = node.planner_stats
        node.get_logger().info(
            f'planner usage — LIN: {st["lin"]}, '
            f'OMPL fallback: {st["ompl_fallback"]}, failed: {st["failed"]}')
        node.destroy_node()
        rclpy.shutdown()
