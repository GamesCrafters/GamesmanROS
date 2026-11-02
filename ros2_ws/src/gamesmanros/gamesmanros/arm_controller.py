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

Collision scene: the board is added to arm_controller's planning scene at
startup, and the pieces are kept in sync with /vision/board_state whenever vision
is running (without vision the board alone is modelled). See scene.py.

Fixed end-effector orientation: gripper pointing down, rotated 45° about Z,
with a -11.25° tilt about X to compensate for the MyCobot wrist angle.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_euler, quaternion_multiply

from moveit.planning import MoveItPy, PlanRequestParameters

from gamesmanros_interfaces.action import MoveArm
from gamesmanros_interfaces.msg import BoardState

from . import scene
from .centers import get_dim
from .robot_control import CoordConverter

BASE_FRAME = 'g_base'  # root link of mycobot_280_moveit_config's URDF
LIN_PIPELINE  = 'pilz_industrial_motion_planner'
LIN_PLANNER   = 'LIN'
OMPL_PIPELINE = 'ompl'
OMPL_PLANNER  = 'RRTConnectkConfigDefault'
LIN_SCALING   = 0.5   # fraction of the Cartesian limits (pilz_cartesian_limits.yaml)
OMPL_SCALING  = 0.3   # fraction of the joint limits (joint_limits.yaml)
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

        self._lin_params = self._make_params(LIN_PIPELINE, LIN_PLANNER, LIN_SCALING)
        self._ompl_params = self._make_params(OMPL_PIPELINE, OMPL_PLANNER, OMPL_SCALING)
        if self._lin_params is None:
            self.get_logger().warn(
                f'{LIN_PIPELINE} pipeline unavailable — running OMPL only. '
                f'Enable it in the moveit_config to get predictable Cartesian motion.')
            self._use_lin_first = False

        # How often each planner actually served a goal.
        self.planner_stats = {'lin': 0, 'ompl_fallback': 0, 'failed': 0}

        self._setup_scene()

        self._action_server = ActionServer(
            self,
            MoveArm,
            'move_arm',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.REJECT,
        )
        self.get_logger().info('ArmController ready')

    # ------------------------------------------------------------------
    # Collision scene (board + pieces)
    # ------------------------------------------------------------------

    def _setup_scene(self):
        self.declare_parameter('game_id', '')
        game = self.get_parameter('game_id').value or scene.DEFAULT_GAME
        dim = get_dim(game)
        self._conv, self._pieces = None, ()
        if not dim:
            self.get_logger().warn(
                f'no board dimensions for {game!r} — planning without a collision scene')
            return
        self._conv = CoordConverter(dim)
        if self._sync_scene([]):
            self.get_logger().info(f'collision scene: {game} board (pieces follow /vision/board_state)')
        self.create_subscription(BoardState, '/vision/board_state', self._on_board_state, 10)

    def _on_board_state(self, msg: BoardState):
        pieces = tuple(sorted((round(d.board_x, 3), round(d.board_y, 3))
                              for d in msg.pieces if d.cell_index >= 0))
        if pieces == self._pieces:
            return
        self._pieces = pieces
        self._sync_scene(list(pieces))

    def _object(self, obj_id, kind, dims, x, y, z) -> CollisionObject:
        prim = SolidPrimitive()
        prim.type = kind
        prim.dimensions = [float(d) for d in dims]
        obj = CollisionObject()
        obj.header.frame_id = BASE_FRAME
        obj.id = obj_id
        obj.operation = CollisionObject.ADD
        obj.primitives.append(prim)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
        pose.orientation.w = 1.0
        obj.primitive_poses.append(pose)
        return obj

    def _sync_scene(self, pieces: list) -> bool:
        """Replace the scene's contents with the board plus these pieces ((x, y) in cell units)."""
        cx, cy, cz, sx, sy, sz = scene.board_box(self._conv)
        objs = [self._object('board', SolidPrimitive.BOX, (sx, sy, sz), cx, cy, cz)]
        for i, (bx, by) in enumerate(pieces):
            px, py, pz, h, r = scene.piece_cylinder(self._conv, bx, by)
            objs.append(self._object(f'piece_{i}', SolidPrimitive.CYLINDER, (h, r), px, py, pz))
        try:
            with self._moveit.get_planning_scene_monitor().read_write() as ps:
                ps.remove_all_collision_objects()
                for o in objs:
                    ps.apply_collision_object(o)
                ps.current_state.update()
            return True
        except Exception as e:   # never let scene bookkeeping take the arm down
            self.get_logger().warn(f'could not update the planning scene: {e}')
            return False

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

    def _make_params(self, pipeline: str, planner: str, scaling: float):
        """Build PlanRequestParameters for one planner, or None if that fails.

        Humble's PlanRequestParameters takes only the MoveItPy instance (it loads
        its defaults from `plan_request_params.*` in moveit_py.yaml), so everything
        specific to this planner is set here.
        """
        try:
            p = PlanRequestParameters(self._moveit)
            p.planning_pipeline = pipeline
            p.planner_id = planner
            p.max_velocity_scaling_factor = scaling
            p.max_acceleration_scaling_factor = scaling
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
            plan = self._arm.plan(parameters=self._lin_params)
            if plan:
                self.planner_stats['lin'] += 1
                return plan, 'LIN'
            self.get_logger().warn(
                f'LIN found no straight-line path to '
                f'({x:.3f}, {y:.3f}, {z:.3f}) — falling back to OMPL')

        # Re-seed the start state; the failed attempt leaves it stale.
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped_msg=pose, pose_link=EEF_LINK)
        plan = (self._arm.plan(parameters=self._ompl_params)
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

        # Humble's signature: execute(group_name, robot_trajectory, blocking=True).
        self._moveit.execute(PLANNING_GROUP, plan_result.trajectory, blocking=True)

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
