#!/usr/bin/env python3
"""
GameManager — ROS 2 lifecycle node owning the UWAPI game loop.

Lifecycle transitions:
  unconfigured → configure  → inactive   (fetches game list, user selects game/variant)
  inactive     → activate   → active     (starts game loop thread)
  active       → deactivate → inactive   (stops game loop thread)

The game loop runs in a daemon thread so it can block on console input for
human turns without stalling the ROS executor.
"""

import threading
import requests

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.action import ActionClient

from gamesmanros_interfaces.action import ExecuteMove
from gamesmanros_interfaces.msg import GameState

UWAPI_BASE = 'https://nyc.cs.berkeley.edu/universal/v1/'


# ---------------------------------------------------------------------------
# Move selection helpers
# ---------------------------------------------------------------------------

# moves is a list of dictionaries, which are the encodings of the responses of UWAPI
def _pick_best_move(moves: list) -> str:
    by_value = {} # a dictonary, whose keys are strings and values are list of strings
    for m in moves:
        by_value.setdefault(m['moveValue'], []).append(m['autoguiMove'])
    for v in ('win', 'draw', 'lose'):
        if v in by_value:
            return by_value[v][0]
    raise RuntimeError('No moves with known value')

def _pick_best_position(moves: list) -> str:
    by_value = {}
    for m in moves:
        by_value.setdefault(m['moveValue'], []).append(m['position'])
    for v in ('win', 'draw', 'lose'):
        if v in by_value:
            return by_value[v][0]
    raise RuntimeError('No positions with known value')

def _human_pick(moves: list, show_values: bool) -> tuple:
    print('\nAvailable moves:')
    for i, m in enumerate(moves):
        parts = m['autoguiMove'].split('_')
        val   = f"  [{m['moveValue']}]" if show_values else ''
        print(f"  {i}: {parts[1]} → {parts[2]}{val}")
    idx = int(input('Choose move index: '))
    return moves[idx]['autoguiMove'], moves[idx]['position']


# ---------------------------------------------------------------------------
# Lifecycle node
# ---------------------------------------------------------------------------

class GameManager(LifecycleNode):
    def __init__(self):
        super().__init__('game_manager')
        self._execute_client: ActionClient | None = None
        self._game_pub = None
        self._game_thread: threading.Thread | None = None
        self._running = False

        # Set after configure
        self._url          = ''
        self._game_id      = ''
        self._human_a      = False
        self._human_b      = False
        self._show_values  = False

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._execute_client = ActionClient(self, ExecuteMove, 'execute_move')
        self._game_pub = self.create_lifecycle_publisher(
            GameState, '/game/state', 10)

        # Interactive setup runs here (before activate) so it doesn't block
        # the executor — on_configure is called from a service callback but
        # the executor is still spinning on other threads.
        try:
            self._interactive_setup()
        except Exception as e:
            self.get_logger().error(f'Setup failed: {e}')
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        super().on_activate(state)
        self._running = True
        self._game_thread = threading.Thread(
            target=self._game_loop, daemon=True)
        self._game_thread.start()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._running = False
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._execute_client:
            self._execute_client.destroy()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._running = False
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Interactive game setup
    # ------------------------------------------------------------------

    def _interactive_setup(self):
        games = requests.get(UWAPI_BASE).json()
        print('\nAvailable games:')
        for i, g in enumerate(games):
            print(f'  {i}: {g["name"]}')
        game_idx = int(input('Pick game index: '))
        self._game_id = games[game_idx]['id']

        url = UWAPI_BASE + self._game_id + '/'
        variants = requests.get(url).json()['variants']
        print('\nVariants:')
        for j, v in enumerate(variants):
            print(f'  {j}: {v["id"]}')
        variant_idx = int(input('Pick variant index: '))
        variant = variants[variant_idx]['id']

        url += variant + '/'
        data = requests.get(url).json()
        self._start_position = data['startPosition']
        self._positions_url  = url + 'positions/?p='

        self._human_a = input('\nPlayer 1 — human or robot? [h/r]: ').strip() == 'h'
        self._human_b = input('Player 2 — human or robot? [h/r]: ').strip() == 'h'
        if self._human_a or self._human_b:
            self._show_values = input('Show move values? [y/n]: ').strip() == 'y'

        self._url = url
        self.get_logger().info(
            f'Game configured: {self._game_id}, variant {variant}')

    # ------------------------------------------------------------------
    # Game loop (runs in daemon thread)
    # ------------------------------------------------------------------

    def _game_loop(self):
        current_pos = self._start_position
        moves = requests.get(self._positions_url + current_pos).json()['moves']
        a_turn = True

        while self._running and moves:
            human = self._human_a if a_turn else self._human_b

            if human:
                move_str, new_pos = _human_pick(moves, self._show_values)
            else:
                move_str = _pick_best_move(moves)
                new_pos  = _pick_best_position(moves)
                self._send_robot_move(move_str, current_pos, new_pos)

            player = 'A' if a_turn else 'B'
            self.get_logger().info(f'Player {player}: {move_str}')

            self._publish_state(new_pos, moves, robot_turn=not human)

            current_pos = new_pos
            moves = requests.get(self._positions_url + current_pos).json()['moves']
            a_turn = not a_turn

        self.get_logger().info('Game over')

    def _send_robot_move(self, move_str: str, current_pos: str, new_pos: str):
        if not self._execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('execute_move action server not available')
            return

        goal = ExecuteMove.Goal()
        goal.game_id          = self._game_id
        goal.move_string      = move_str
        goal.current_position = current_pos
        goal.new_position     = new_pos

        future = self._execute_client.send_goal_async(goal)
        # Block the game-loop thread until the move completes
        import concurrent.futures
        event = threading.Event()
        result_holder = {}

        def done(f):
            gh = f.result()
            if not gh.accepted:
                result_holder['ok'] = False
                event.set()
                return
            res_future = gh.get_result_async()
            res_future.add_done_callback(
                lambda rf: (result_holder.update({'ok': rf.result().result.success}),
                            event.set()))

        future.add_done_callback(done)
        event.wait()

        if not result_holder.get('ok', False):
            self.get_logger().warn(f'Move {move_str} reported failure')

    def _publish_state(self, position: str, moves: list, robot_turn: bool):
        if self._game_pub is None:
            return
        msg = GameState()
        msg.position        = position
        msg.available_moves = [m['autoguiMove'] for m in moves]
        msg.is_robot_turn   = robot_turn
        msg.game_over       = len(moves) == 0
        self._game_pub.publish(msg)


def main():
    rclpy.init()
    node = GameManager()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Trigger configure → activate automatically on startup
    node.trigger_configure()
    node.trigger_activate()

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
