#!/usr/bin/env python3
"""
Reachability sweep — fires every board cell at /move_arm and tabulates results.

Answers two questions nobody has measured yet:
  1. Which board cells can the arm actually reach, at both z heights?
  2. How often does LIN fail and fall back to OMPL?

Run it against the real arm_controller with a MoveIt config loaded. It also runs
against mock_hardware, which is useful for checking the script itself.

    ros2 run gamesmanros reach_sweep
    ros2 run gamesmanros reach_sweep --ros-args -p game:=dodgem -p mode:=pairs

Parameters
  game        game id from centers.py              (default dodgem)
  mode        'cells' or 'pairs'                   (default cells)
  settle      seconds to pause between goals       (default 0.0)

'cells' sends one goal per cell per height — fast, answers reachability.
'pairs' walks every ordered cell-to-cell transition at travel height, which is
what actually exercises LIN's straight-line constraint. It is O(n^2), so on a
16-cell board that is 240 moves.
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from gamesmanros_interfaces.action import MoveArm
from .centers import get_centers, get_dim
from .robot_control import CoordConverter, PICKUP_Z_MM, LIFT_Z_MM


class ReachSweep(Node):
    def __init__(self):
        super().__init__('reach_sweep')
        self.declare_parameter('game', 'dodgem')
        self.declare_parameter('mode', 'cells')
        self.declare_parameter('settle', 0.0)

        self.game = self.get_parameter('game').value
        self.mode = self.get_parameter('mode').value
        self.settle = self.get_parameter('settle').value

        self._client = ActionClient(self, MoveArm, 'move_arm')

    def send(self, x, y, z):
        """Blocking send. Returns (ok, message)."""
        goal = MoveArm.Goal()
        goal.x, goal.y, goal.z = float(x), float(y), float(z)
        fut = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, 'goal rejected'
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        res = rf.result().result
        return res.success, (res.message or '')

    def run(self):
        centers = get_centers(self.game)
        dim = get_dim(self.game)
        if not centers or not dim:
            self.get_logger().error(f'no centers/dim for {self.game!r}')
            return 1

        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('/move_arm not available — is arm_controller running?')
            return 1

        conv = CoordConverter(dim)
        print(f'\nReachability sweep — {self.game}, dim={dim}, {len(centers)} cells, mode={self.mode}\n')

        if self.mode == 'pairs':
            return self._sweep_pairs(conv, centers)
        return self._sweep_cells(conv, centers)

    def _sweep_cells(self, conv, centers):
        print(f'{"cell":>5}{"x":>9}{"y":>8}{"reach":>8}   {"pick(135)":<14}{"travel(185)":<14}')
        print('-' * 62)
        bad = []
        for i, svg in enumerate(centers):
            x, y, _ = conv.svg_to_meters(svg, PICKUP_Z_MM)
            reach = math.hypot(x, y) * 1000
            row = []
            for z_mm in (PICKUP_Z_MM, LIFT_Z_MM):
                _, _, z = conv.svg_to_meters(svg, z_mm)
                ok, msg = self.send(x, y, z)
                row.append(f'{"OK" if ok else "FAIL"} {msg}'.strip()[:13])
                if not ok:
                    bad.append((i, z_mm))
                if self.settle:
                    time.sleep(self.settle)
            print(f'{i:>5}{x:>9.4f}{y:>8.4f}{reach:>8.1f}   {row[0]:<14}{row[1]:<14}')

        print(f'\n{len(centers)*2 - len(bad)}/{len(centers)*2} goals planned.')
        if bad:
            print('unreachable: ' + ', '.join(f'cell {c}@{z:.0f}mm' for c, z in bad))
        return 0

    def _sweep_pairs(self, conv, centers):
        n = len(centers)
        print(f'walking {n*(n-1)} ordered transitions at travel height {LIFT_Z_MM:.0f} mm\n')
        fails, lin, ompl = [], 0, 0
        for a in range(n):
            ax, ay, az = conv.svg_to_meters(centers[a], LIFT_Z_MM)
            if not self.send(ax, ay, az)[0]:
                continue                        # cannot stage here; skip its row
            for b in range(n):
                if a == b:
                    continue
                bx, by, bz = conv.svg_to_meters(centers[b], LIFT_Z_MM)
                ok, msg = self.send(bx, by, bz)
                if not ok:
                    fails.append((a, b))
                elif 'OMPL' in msg:
                    ompl += 1
                else:
                    lin += 1
                if self.settle:
                    time.sleep(self.settle)
            print(f'  from cell {a:>2}: done')

        total = lin + ompl + len(fails)
        print(f'\ntransitions: {total}')
        if total:
            print(f'  LIN succeeded   {lin:>5}  ({100*lin/total:.1f}%)')
            print(f'  OMPL fallback   {ompl:>5}  ({100*ompl/total:.1f}%)')
            print(f'  both failed     {len(fails):>5}  ({100*len(fails)/total:.1f}%)')
        if fails:
            print('\nfailed transitions: ' +
                  ', '.join(f'{a}->{b}' for a, b in fails[:40]))
        return 0


def main():
    rclpy.init()
    node = ReachSweep()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)
