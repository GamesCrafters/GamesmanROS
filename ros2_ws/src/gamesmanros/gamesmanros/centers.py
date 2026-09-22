"""
Board center coordinates in SVG space for each game.

Coordinates are normalized so the board fits within the robot's reachable area.
Max values are board_dim in each axis; the RobotControl class applies the
scaling and offset to convert to real-world meters.
"""

import math

_centers = {
    "1dchess":       [[0.5 + i, 0.5] for i in range(8)],
    "allqueenschess": [[(i % 5 + 0.5) / 5, (i // 5 + 0.5) / 5] for i in range(25)],
    "dawsonschess":  [[0.5,1.5],[1.5,1.5],[2.5,1.5],[3.5,1.5],[4.5,2.5],[5.5,2.5]],
    "dao":           [[(i % 4 + 0.5) / 4, (i // 4 + 0.5) / 4] for i in range(16)],
    "dodgem":        [[(i % 4 + 0.5), (i // 4 + 0.5)] for i in range(16)],
    "dinododgem":    [[i // 5 + 0.5, 5.5 - (i % 5)] for i in range(25)] + [
                         [1.5,0.5],[2.5,0.5],[3.5,0.5],[4.5,0.5],
                         [5.5,1.5],[5.5,2.5],[5.5,3.5],[5.5,4.5],
                     ],
    "dragonsandswans": (
        [[(i % 4 * 10 + 5) / 35, (i // 4 * 10 + 5) / 46] for i in range(16)] +
        [[28.7/35, 43/46], [30.2/35, 43/46], [28.7/35, 46/46], [30.2/35, 46/46]]
    ),
    "jan":   [[i % 4, i // 4] for i in range(16)],
    "joust": [[0.5+col, 0.5+row] for row in range(4) for col in range(4)],
}

_dim = {
    "1dchess":       8,
    "allqueenschess": 5,
    "dawsonschess":  5,
    "dao":           4,
    "dodgem":        3,
    "dinododgem":    6,
    "jan":           3,
    "joust":         4,
}

_pickup = {
    # game_id: [[x, y], ...]  — SVG coords of off-board piece stacks
}

_capture = {
    "1dchess": [5.5, 2.5],
}

# Maps AR marker frame names to current board position indices (mutable at runtime)
_ar_tracker = {
    "dodgem": {
        "ar_marker_16": 4,
        "ar_marker_13": 8,
        "ar_marker_6":  13,
        "ar_marker_7":  14,
    }
}


def get_centers(game):
    return _centers.get(game)

def get_dim(game):
    return _dim.get(game)

def get_pickup(game):
    return _pickup.get(game)

def get_capture(game):
    return _capture.get(game)

# ar_tracker is a nested 2-D dictionary, so we must index into the outer dict thru the game key & get it's corresponding items (AR frame and index)
def get_piece_ar_frame(game, index):
    for frame, idx in _ar_tracker.get(game, {}).items():
        if idx == index:
            return frame
    return None

def get_ar_tracker(game):
    return _ar_tracker.get(game, {})

def set_piece_ar_frame(game, end_index, frame):
    if game in _ar_tracker:
        _ar_tracker[game][frame] = end_index

# Returns a coordinate in the form of a 2 item list
def process_ar_location(game, get_pose_fn, start_index, end_index):
    frame = get_piece_ar_frame(game, start_index)
    set_piece_ar_frame(game, end_index, frame)
    return get_pose_fn(frame)
