#!/usr/bin/env python3
"""
ExecuteMove action server — owns the Type1-Type11 game class taxonomy.

Decodes UWAPI move strings and drives the arm + gripper via action clients.

Coordinate pipeline:
  UWAPI centers → normalize (u, v) in centers.py → robot meters in coords.py

Capture and pickup are shared physical workspace points from calibration.py.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from gamesmanros_interfaces.action import ExecuteMove, MoveArm, MoveGripper

from .calibration import (
    BOARD_HEIGHT_MM,
    BOARD_WIDTH_MM,
    FLIP_X,
    FLIP_Y,
    LIFT_Z_MM,
    ORIGIN_X_MM,
    ORIGIN_Y_MM,
    PICKUP_Z_MM,
    get_capture,
    get_pickup,
)
from .centers import GeometryError, get_centers
from .coords import CoordinateError, CoordConverter


# ---------------------------------------------------------------------------
# Motion sequencer — fully async, no spin_until_future_complete
# ---------------------------------------------------------------------------

class MotionSequencer:
    def __init__(
        self,
        arm_client: ActionClient,
        gripper_client: ActionClient,
        logger=None,
        dry_run: bool = False,
    ):
        self._arm = arm_client
        self._gripper = gripper_client
        self._logger = logger
        self.dry_run = dry_run

    async def _send_arm(self, x, y, z) -> bool:
        if self.dry_run:
            if self._logger:
                self._logger.info(f"[dry-run] arm → ({x:.4f}, {y:.4f}, {z:.4f})")
            return True
        goal = MoveArm.Goal()
        goal.x, goal.y, goal.z = x, y, z
        gh = await self._arm.send_goal_async(goal)
        if not gh.accepted:
            return False
        result = await gh.get_result_async()
        return result.result.success

    async def _send_gripper(self, open_: bool) -> bool:
        if self.dry_run:
            if self._logger:
                self._logger.info(f"[dry-run] gripper {'open' if open_ else 'close'}")
            return True
        goal = MoveGripper.Goal()
        goal.open = open_
        gh = await self._gripper.send_goal_async(goal)
        if not gh.accepted:
            return False
        result = await gh.get_result_async()
        return result.result.success

    async def pick_and_place(self, conv: CoordConverter, before, after) -> bool:
        try:
            pickup_z_mm = conv.pickup_z * 1000.0
            bx, by, bz = conv.to_meters(before, pickup_z_mm)
            ax, ay, az = conv.to_meters(after, pickup_z_mm)
        except CoordinateError as exc:
            if self._logger:
                self._logger.error(f"Rejected motion target: {exc}")
            return False
        lz = conv.lift_z

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
    def __init__(self, game: str, seq: MotionSequencer,
                 centers: list[list[float]], conv: CoordConverter):
        self.game = game
        self.seq = seq
        self.centers = centers
        self.pickup = get_pickup()
        self.capture = get_capture()
        self.conv = conv


class Type1(BaseType):
    """Place: pick from the shared off-board stack, place at target cell."""

    def __init__(self, game, seq, centers, conv):
        super().__init__(game, seq, centers, conv)
        self.counter = 0

    async def process_move(self, move: str, positions=None) -> bool:
        parts = move.split("_")
        end_index = int(parts[2])
        before = self.pickup[self.counter]
        after = self.centers[end_index]
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
        super().__init__("robot_control")

        cb_group = ReentrantCallbackGroup()

        self.declare_parameter("board_width_mm", BOARD_WIDTH_MM)
        self.declare_parameter("board_height_mm", BOARD_HEIGHT_MM)
        self.declare_parameter("origin_x_mm", ORIGIN_X_MM)
        self.declare_parameter("origin_y_mm", ORIGIN_Y_MM)
        self.declare_parameter("flip_x", FLIP_X)
        self.declare_parameter("flip_y", FLIP_Y)
        self.declare_parameter("pickup_z_mm", PICKUP_Z_MM)
        self.declare_parameter("lift_z_mm", LIFT_Z_MM)
        self.declare_parameter("dry_run", False)

        self._conv = CoordConverter(
            board_width_mm=self.get_parameter("board_width_mm").value,
            board_height_mm=self.get_parameter("board_height_mm").value,
            origin_x_mm=self.get_parameter("origin_x_mm").value,
            origin_y_mm=self.get_parameter("origin_y_mm").value,
            flip_x=self.get_parameter("flip_x").value,
            flip_y=self.get_parameter("flip_y").value,
            pickup_z_mm=self.get_parameter("pickup_z_mm").value,
            lift_z_mm=self.get_parameter("lift_z_mm").value,
        )

        self._arm_client = ActionClient(
            self, MoveArm, "move_arm", callback_group=cb_group)
        self._gripper_client = ActionClient(
            self, MoveGripper, "move_gripper", callback_group=cb_group)

        self._action_server = ActionServer(
            self,
            ExecuteMove,
            "execute_move",
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.REJECT,
            callback_group=cb_group,
        )

        self._board_state: str = ""
        self.create_subscription(
            String, "/vision/board_state", self._on_board_state, 10,
            callback_group=cb_group)

        dry_run = bool(self.get_parameter("dry_run").value)
        self._seq = MotionSequencer(
            self._arm_client, self._gripper_client,
            logger=self.get_logger(), dry_run=dry_run)
        self._game_instance = None
        self._current_game = None

        self.get_logger().info(
            "RobotControl ready "
            f"(board {self._conv.board_width_mm}×{self._conv.board_height_mm} mm"
            f"{', dry_run' if dry_run else ''})"
        )

    def _on_board_state(self, msg: String):
        self._board_state = msg.data

    def _get_game_instance(self, game_id: str, variant_id: str):
        try:
            centers = get_centers(game_id, variant_id)
        except GeometryError as exc:
            self.get_logger().error(str(exc))
            return None

        game_key = (game_id, variant_id)
        if self._current_game != game_key:
            cls = _get_type_class(game_id)
            if cls is None:
                self.get_logger().error(f"Unknown game: {game_id}")
                return None
            self._game_instance = cls(game_id, self._seq, centers, self._conv)
            self._current_game = game_key
        elif self._game_instance is not None:
            self._game_instance.centers = centers
        return self._game_instance

    async def _execute(self, goal_handle):
        req = goal_handle.request
        result = ExecuteMove.Result()

        feedback = ExecuteMove.Feedback()
        feedback.status = f"Processing: {req.move_string}"
        goal_handle.publish_feedback(feedback)

        instance = self._get_game_instance(
            req.game_id, req.variant_id)
        if instance is None:
            result.success = False
            goal_handle.abort()
            return result

        if self._board_state:
            self.get_logger().info(f"Board before move: {self._board_state}")

        positions = (req.current_position, req.new_position)
        ok = await instance.process_move(req.move_string, positions)

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
