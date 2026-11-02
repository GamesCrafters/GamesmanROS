#!/usr/bin/env python3
"""
Collision-scene geometry (scene.py): the board slab and piece cylinders that
arm_controller adds to MoveIt's planning scene.

    cd src/gamesmanros && python3 -m pytest test/test_scene.py -v
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

pytest.importorskip('rclpy')
pytest.importorskip('gamesmanros_interfaces')

from gamesmanros import scene                                   # noqa: E402
from gamesmanros.centers import get_centers, get_dim            # noqa: E402
from gamesmanros.robot_control import CoordConverter, PICKUP_Z_MM   # noqa: E402


def dodgem():
    return CoordConverter(get_dim('dodgem')), get_centers('dodgem')


def test_board_box_covers_every_cell_and_pieces_sit_on_it():
    conv, centers = dodgem()
    cx, cy, cz, sx, sy, sz = scene.board_box(conv)
    for bx, by in centers:
        px, py, pz, h, r = scene.piece_cylinder(conv, bx, by)
        assert abs(px - cx) <= sx / 2 and abs(py - cy) <= sy / 2   # inside the footprint
        assert pz - h / 2 == pytest.approx(cz + sz / 2)            # resting on the top face


def test_board_footprint_matches_the_calibrated_board_size():
    conv, _ = dodgem()
    _, _, _, sx, sy, _ = scene.board_box(conv)
    assert sx == pytest.approx(0.150) and sy == pytest.approx(0.150)


def test_pieces_stay_below_the_pick_height():
    """A piece must be lower than where the flange grips it, or every pick collides."""
    assert scene.BOARD_THICKNESS_MM + scene.PIECE_HEIGHT_MM < PICKUP_Z_MM
