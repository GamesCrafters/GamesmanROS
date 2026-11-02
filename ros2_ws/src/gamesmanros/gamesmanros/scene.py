"""
Collision-scene geometry: the board and the pieces on it, as plain numbers.

arm_controller turns these into MoveIt collision objects so planners treat the
board and pieces as obstacles for the arm's links. All values are metres in the
arm's base frame and come from the SAME CoordConverter the pick-and-place
poses use, so the objects line up with where the arm is actually sent.

Only the arm's own links collide with these. The gripper is not in the URDF, so
descending onto a piece does not trip over the piece it is about to grab.

TODO(measure) — the three heights below are placeholders. Measure the real
board and pieces; they decide how much clearance the planners enforce.
"""

DEFAULT_GAME       = 'dodgem'   # same default as reach_sweep
BOARD_THICKNESS_MM = 10.0       # board top surface above the robot base plane
PIECE_HEIGHT_MM    = 30.0
PIECE_RADIUS_MM    = 12.0


def board_box(conv):
    """(cx, cy, cz, sx, sy, sz) of the board slab. Footprint = the whole playing area."""
    x0, y0, _ = conv.svg_to_meters([0, 0], 0.0)
    x1, y1, _ = conv.svg_to_meters([conv.dim, conv.dim], 0.0)
    t = BOARD_THICKNESS_MM / 1000.0
    return ((x0 + x1) / 2, (y0 + y1) / 2, t / 2,
            abs(x1 - x0), abs(y1 - y0), t)


def piece_cylinder(conv, board_x, board_y):
    """(cx, cy, cz, height, radius) of a piece seen at (board_x, board_y) in cell units."""
    x, y, _ = conv.svg_to_meters([board_x, board_y], 0.0)
    h = PIECE_HEIGHT_MM / 1000.0
    return x, y, BOARD_THICKNESS_MM / 1000.0 + h / 2, h, PIECE_RADIUS_MM / 1000.0
