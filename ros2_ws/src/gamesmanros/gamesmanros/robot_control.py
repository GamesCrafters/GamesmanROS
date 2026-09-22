#!/usr/bin/env python3
"""
ExecuteMove action server — owns the Type1-Type11 game class taxonomy.

Decodes UWAPI move strings and drives the arm + gripper via action clients,
replacing the blocking direct calls from the ROS 1 robotControl.py.

Coordinate pipeline (preserved from ROS 1):
  SVG index → svg_to_meters() affine flip → scale + offset → meters → arm_controller

Fixes over original version:
  - All motion calls are async (eliminates spin_until_future_complete deadlock)
  - game_id comes from the ExecuteMove goal, not a ROS param
  - Subscribes to /vision/board_state for live board state monitoring
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from gamesmanros_interfaces.action import ExecuteMove, MoveArm, MoveGripper
from .centers import get_centers, get_pickup, get_capture, get_dim

BOARD_SIZE_MM = 150.0
Y_OFFSET_MM   = 100.0
PICKUP_Z_MM   = 135.0
LIFT_Z_MM     = 185.0


# ---------------------------------------------------------------------------
# Coordinate conversion
# ---------------------------------------------------------------------------

class CoordConverter:
    def __init__(self, dim: int):
        self._dim     = dim
        self.scaling  = BOARD_SIZE_MM / dim
        self.x_offset = BOARD_SIZE_MM / 2 + 50.0
        self.y_offset = Y_OFFSET_MM
        self.pickup_z = PICKUP_Z_MM / 1000.0
        self.lift_z   = LIFT_Z_MM   / 1000.0

    def svg_to_meters(self, svg: list, z_mm: float) -> tuple:
        """Convert SVG [col, row] to robot base-frame (x, y, z) in meters."""
        # Flip Y: SVG row 0 is top of board, robot row 0 is closest to base
        flipped_row = abs(-svg[1] + (self._dim + 1))
        x = (svg[0] * self.scaling - self.x_offset) / 1000.0
        y = (flipped_row * self.scaling + self.y_offset) / 1000.0
        z = z_mm / 1000.0
        return x, y, z


# ---------------------------------------------------------------------------
# Motion sequencer — fully async, no spin_until_future_complete
# ---------------------------------------------------------------------------

class MotionSequencer:
    def __init__(self, arm_client: ActionClient, gripper_client: ActionClient):
        self._arm     = arm_client
        self._gripper = gripper_client

    async def _send_arm(self, x, y, z) -> bool:
        goal            = MoveArm.Goal()
        goal.x, goal.y, goal.z = x, y, z
        gh              = await self._arm.send_goal_async(goal)
        if not gh.accepted:
            return False
        result          = await gh.get_result_async()
        return result.result.success

    async def _send_gripper(self, open_: bool) -> bool:
        goal      = MoveGripper.Goal()
        goal.open = open_
        gh        = await self._gripper.send_goal_async(goal)
        if not gh.accepted:
            return False
        result    = await gh.get_result_async()
        return result.result.success

    async def pick_and_place(self, conv: CoordConverter,
                             before_svg: list, after_svg: list) -> bool:
        bx, by, bz = conv.svg_to_meters(before_svg, PICKUP_Z_MM)
        ax, ay, az = conv.svg_to_meters(after_svg,  PICKUP_Z_MM)
        lz         = conv.lift_z

        ok = True
        ok = ok and await self._send_gripper(open_=True)   # open before approaching
        ok = ok and await self._send_arm(bx, by, lz)       # hover above piece
        ok = ok and await self._send_arm(bx, by, bz)       # descend onto piece
        ok = ok and await self._send_gripper(open_=False)  # grip
        ok = ok and await self._send_arm(bx, by, lz)       # lift
        ok = ok and await self._send_arm(ax, ay, lz)       # travel to target column
        ok = ok and await self._send_arm(ax, ay, az)       # descend to target
        ok = ok and await self._send_gripper(open_=True)   # release
        ok = ok and await self._send_arm(ax, ay, lz)       # retreat upward
        return ok


# ---------------------------------------------------------------------------
# Type class hierarchy (preserves ROS 1 taxonomy)
# ---------------------------------------------------------------------------

class BaseType:
    def __init__(self, game: str, seq: MotionSequencer):
        self.game    = game
        self.seq     = seq
        self.centers = get_centers(game)
        self.pickup  = get_pickup(game)
        self.capture = get_capture(game)
        self.dim     = get_dim(game)
        self.conv    = CoordConverter(self.dim)


class Type1(BaseType):
    """Place: pick from off-board stack, place at target cell."""
    def __init__(self, game, seq):
        super().__init__(game, seq)
        self.counter = 0

    async def process_move(self, move: str, positions=None) -> bool:
        parts     = move.split('_')
        end_index = int(parts[2])
        before    = self.pickup[0][self.counter]
        after     = self.centers[end_index]
        self.counter += 1
        return await self.seq.pick_and_place(self.conv, before, after)


class Type2(BaseType):
    """Captures only."""
    async def process_move(self, move: str, positions=None) -> bool:
        parts       = move.split('_')
        start_index = int(parts[1])
        end_index   = int(parts[2])
        start_pos, end_pos = positions
        if start_pos[end_index] != end_pos[end_index] and start_pos[end_index] != '-':
            ok = await self.seq.pick_and_place(
                self.conv, self.centers[end_index], self.capture)
            return ok and await self.seq.pick_and_place(
                self.conv, self.centers[start_index], self.centers[end_index])
        return await self.seq.pick_and_place(
            self.conv, self.centers[start_index], self.centers[end_index])


class Type4(BaseType):
    """Re-arrange (slide piece from cell to cell)."""
    async def process_move(self, move: str, positions=None) -> bool:
        parts       = move.split('_')
        start_index = int(parts[1])
        end_index   = int(parts[2])
        return await self.seq.pick_and_place(
            self.conv, self.centers[start_index], self.centers[end_index])


class Type6(BaseType):
    """Re-arrange + Removal."""
    async def process_move(self, move: str, positions=None) -> bool:
        parts       = move.split('_')
        start_index = int(parts[1])
        end_index   = int(parts[2])
        return await self.seq.pick_and_place(
            self.conv, self.centers[start_index], self.centers[end_index])


class Type7(BaseType):
    """Re-arrange + Capture."""
    async def process_move(self, move: str, positions=None) -> bool:
        parts       = move.split('_')
        start_index = int(parts[1])
        end_index   = int(parts[2])
        start_pos, end_pos = positions
        if start_pos[end_index] != end_pos[end_index] and start_pos[end_index] != '-':
            ok = await self.seq.pick_and_place(
                self.conv, self.centers[end_index], self.capture)
            return ok and await self.seq.pick_and_place(
                self.conv, self.centers[start_index], self.centers[end_index])
        return await self.seq.pick_and_place(
            self.conv, self.centers[start_index], self.centers[end_index])


_GAME_TYPES = {
    "Type1": (Type1, ["dawsonschess"]),
    "Type4": (Type4, ["3spot", "allqueenschess", "beeline", "change", "dao",
                      "fivefieldkono", "foxandhounds", "hareandhounds",
                      "jan", "joust", "hobaggonu"]),
    "Type6": (Type6, ["dinododgem", "dodgem"]),
    "Type7": (Type7, ["1dchess"]),
}

def _get_type_class(game_id: str):
    for cls, games in _GAME_TYPES.values():
        if game_id in games:
            return cls
    return None


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')

        cb_group = ReentrantCallbackGroup()

        self._arm_client = ActionClient(
            self, MoveArm, 'move_arm', callback_group=cb_group)
        self._gripper_client = ActionClient(
            self, MoveGripper, 'move_gripper', callback_group=cb_group)

        self._action_server = ActionServer(
            self,
            ExecuteMove,
            'execute_move',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.REJECT,
            callback_group=cb_group,
        )

        # Latest board occupancy from vision_node — logged before each move
        self._board_state: str = ''
        self.create_subscription(
            String, '/vision/board_state', self._on_board_state, 10,
            callback_group=cb_group)

        self._seq           = MotionSequencer(self._arm_client, self._gripper_client)
        self._game_instance = None
        self._current_game  = None

        self.get_logger().info('RobotControl ready')

    def _on_board_state(self, msg: String):
        self._board_state = msg.data

    def _get_game_instance(self, game_id: str):
        if self._current_game != game_id:
            cls = _get_type_class(game_id)
            if cls is None:
                self.get_logger().error(f'Unknown game: {game_id}')
                return None
            self._game_instance = cls(game_id, self._seq)
            self._current_game  = game_id
        return self._game_instance

    async def _execute(self, goal_handle):
        req    = goal_handle.request
        result = ExecuteMove.Result()

        feedback        = ExecuteMove.Feedback()
        feedback.status = f'Processing: {req.move_string}'
        goal_handle.publish_feedback(feedback)

        # game_id now comes directly from the goal (not a ROS param hack)
        instance = self._get_game_instance(req.game_id)
        if instance is None:
            result.success = False
            goal_handle.abort()
            return result

        if self._board_state:
            self.get_logger().info(f'Board before move: {self._board_state}')

        positions = (req.current_position, req.new_position)
        ok        = await instance.process_move(req.move_string, positions)

        result.success = ok
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = RobotControl()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
