#!/usr/bin/env python3
"""
FollowJointTrajectory action server for the real MyCobot.

arm_controller's MoveItPy executes planned trajectories through
moveit_simple_controller_manager, which sends them to the action
`joint_trajectory_controller/follow_joint_trajectory`. In the mock demo
ros2_control serves that; on the real arm THIS node does, by streaming each
trajectory point to the driver on /mycobot/angles_goal. A port of the ROS 1
joint_trajectory.py, with two changes:

  * points are sent at their planned times (time_from_start), not a fixed 30 Hz,
    so the velocity scaling set in arm_controller is respected;
  * after the last point it waits for the arm to arrive (/mycobot/angles_real)
    before reporting success — the arm lags its commands, and the next gripper
    or arm command must not start while it is still moving.

Parameters
  speed                   MycobotSetAngles speed, 1-100        (default 40, as ROS 1)
  arrival_tolerance_deg   "arrived" = every joint within this  (default 3.0)
  settle_timeout          seconds to wait for arrival          (default 6.0)
"""

import math
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from mycobot_interfaces.msg import MycobotAngles, MycobotSetAngles

JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]


def order_positions(names, positions):
    """Positions reordered into JOINT_NAMES order. Raises ValueError on a mismatch."""
    if sorted(names) != sorted(JOINT_NAMES):
        raise ValueError(f'unexpected joints {list(names)}')
    by_name = dict(zip(names, positions))
    return [by_name[n] for n in JOINT_NAMES]


def to_degrees(radians):
    return [math.degrees(r) for r in radians]


def arrived(current, target, tolerance_deg):
    return all(abs(c - t) <= tolerance_deg for c, t in zip(current, target))


def point_time(point):
    return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9


class JointTrajectoryActionServer(Node):
    def __init__(self):
        super().__init__('joint_trajectory')
        self.declare_parameter('speed', 40)
        self.declare_parameter('arrival_tolerance_deg', 3.0)
        self.declare_parameter('settle_timeout', 6.0)

        self._real = None      # latest /mycobot/angles_real, degrees
        self._pub = self.create_publisher(MycobotSetAngles, '/mycobot/angles_goal', 10)
        # Separate reentrant group so angle updates keep arriving while a goal runs.
        self.create_subscription(
            MycobotAngles, '/mycobot/angles_real', self._on_real, 10,
            callback_group=ReentrantCallbackGroup())

        self._server = ActionServer(
            self, FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.REJECT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info('JointTrajectoryActionServer ready')

    def _on_real(self, msg: MycobotAngles):
        self._real = [msg.joint_1, msg.joint_2, msg.joint_3,
                      msg.joint_4, msg.joint_5, msg.joint_6]

    def _on_goal(self, goal_request):
        traj = goal_request.trajectory
        if not traj.points:
            self.get_logger().warn('rejecting an empty trajectory')
            return GoalResponse.REJECT
        try:
            order_positions(traj.joint_names, traj.points[0].positions)
        except ValueError as e:
            self.get_logger().error(f'rejecting trajectory: {e}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _send(self, degrees, speed):
        msg = MycobotSetAngles()
        (msg.joint_1, msg.joint_2, msg.joint_3,
         msg.joint_4, msg.joint_5, msg.joint_6) = degrees
        msg.speed = speed
        self._pub.publish(msg)

    def _wait_for_arrival(self, target):
        tol = float(self.get_parameter('arrival_tolerance_deg').value)
        deadline = time.monotonic() + float(self.get_parameter('settle_timeout').value)
        while time.monotonic() < deadline:
            if self._real is not None and arrived(self._real, target, tol):
                return True
            time.sleep(0.1)
        return False

    def _execute(self, goal_handle):
        traj = goal_handle.request.trajectory
        speed = int(self.get_parameter('speed').value)
        names = list(traj.joint_names)
        self.get_logger().info(f'executing {len(traj.points)} points')

        start = time.monotonic()
        target = None
        for point in traj.points:
            wait = point_time(point) - (time.monotonic() - start)
            if wait > 0:
                time.sleep(wait)
            target = to_degrees(order_positions(names, point.positions))
            self._send(target, speed)

        if not self._wait_for_arrival(target):
            self.get_logger().warn(
                'arm did not reach the final point in time '
                '(is mycobot_topics running and publishing /mycobot/angles_real?)')

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main():
    rclpy.init()
    node = JointTrajectoryActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
