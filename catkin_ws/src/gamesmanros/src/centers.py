"""
Get Centers for the Robot

Objective is to standardize centers for the games by ensuring that the maximum values
for the centers are (1,1) and the minimum is (0,0).

NOTE: Leave the coordinates in SVG coordinate space

{id: [list of centers]}
id: Must Match GameID from UWAPI init functions
[list of centers]: Similar to UWAPI imageautogui data, provide a list of centers
                   with a max ratio of 1:1. All values should be in [0, dim] range
                   so that dividing by get_dim() normalizes them to [0, 1].
"""


def get_centers(game):
    data = {
        # 8 squares in a row, x goes 0.5 -> 7.5, y fixed at 0.5
        "1dchess": [[0.5 + i, 0.5] for i in range(8)],

        # 5x5 grid normalized to [0,1] already (divided by 5 inline)
        "allqueenschess": [[(i % 5 + 0.5) / 5, (i // 5 + 0.5) / 5] for i in range(25)],

        # 6 squares, coords in [0, 6] range (get_dim=6)
        "dawsonschess": [[0.5, 1.5], [1.5, 1.5], [2.5, 1.5], [3.5, 1.5], [4.5, 2.5], [5.5, 2.5]],

        # 4x4 grid normalized to [0,1] already (divided by 4 inline)
        "dao": [[(i % 4 + 0.5) / 4, (i // 4 + 0.5) / 4] for i in range(16)],

        # 4x4 grid, coords in [0, 4] range (get_dim=4)
        "dodgem": [[(i % 4 + 0.5), (i // 4 + 0.5)] for i in range(16)],

        # 5x5 grid + extra edge squares, coords in [0, 6] range (get_dim=6)
        "dinododgem": [[i // 5 + 0.5, 5.5 - (i % 5)] for i in range(25)] + [
            [1.5, 0.5], [2.5, 0.5], [3.5, 0.5], [4.5, 0.5],
            [5.5, 1.5], [5.5, 2.5], [5.5, 3.5], [5.5, 4.5]
        ],

        # Already normalized to [0,1]
        "dragonsandswans": [[(i % 4 * 10 + 5) / 35, (i // 4 * 10 + 5) / 46] for i in range(16)] + [
            [28.7 / 35, 43 / 46], [30.2 / 35, 43 / 46],
            [28.7 / 35, 46 / 46], [30.2 / 35, 46 / 46]
        ],

        # 4x4 grid, coords in [0, 4] range (get_dim=4)
        # Fixed: was missing +0.5 offset so pieces were on grid lines not centers
        "jan": [[(i % 4 + 0.5), (i // 4 + 0.5)] for i in range(16)],

        # 4x4 grid, coords in [0, 4] range (get_dim=4)
        "joust": [
            [0.5, 0.5], [1.5, 0.5], [2.5, 0.5], [3.5, 0.5],
            [0.5, 1.5], [1.5, 1.5], [2.5, 1.5], [3.5, 1.5],
            [0.5, 2.5], [1.5, 2.5], [2.5, 2.5], [3.5, 2.5],
            [0.5, 3.5], [1.5, 3.5], [2.5, 3.5], [3.5, 3.5]
        ],

        # 5 points, coords in [0, 100] range (get_dim=100)
        "ponghauki": [[25, 25], [75, 25], [50, 50], [25, 75], [75, 75]],

        # 4x4 grid, coords in [0, 4] range (get_dim=4)
        "beeline": [[(i % 4 + 0.5), (i // 4 + 0.5)] for i in range(16)],

        # 4x4 grid, coords in [0, 4] range (get_dim=4)
        "change": [[(i % 4 + 0.5), (i // 4 + 0.5)] for i in range(16)],

        # 4x4 grid, coords in [0, 4] range (get_dim=4)
        "fivefieldkono": [[(i % 4 + 0.5), (i // 4 + 0.5)] for i in range(16)],

        # 4x4 grid, coords in [0, 4] range (get_dim=4)
        "foxandhounds": [[(i % 4 + 0.5), (i // 4 + 0.5)] for i in range(16)],

        # 4x4 grid, coords in [0, 4] range (get_dim=4)
        "hareandhounds": [[(i % 4 + 0.5), (i // 4 + 0.5)] for i in range(16)],

        # 4x4 grid, coords in [0, 4] range (get_dim=4)
        "hobaggonu": [[(i % 4 + 0.5), (i // 4 + 0.5)] for i in range(16)],

        # 3-spot: 3 positions in a row (get_dim=3)
        "3spot": [[0.5, 0.5], [1.5, 0.5], [2.5, 0.5]],

        # 4x4 grid, coords in [0, 4] range (get_dim=4)
        "4squaretictactoe": [[(i % 4 + 0.5), (i // 4 + 0.5)] for i in range(16)],
    }
    if game not in data:
        rospy.logwarn("get_centers: no centers defined for game '%s'" % game) if _ros_ok() else print("get_centers: no centers defined for game '%s'" % game)
        return None
    return data[game]


def _ros_ok():
    try:
        import rospy
        return rospy.core.is_initialized()
    except Exception:
        return False


ar_tracker = {
    "dodgem": {"ar_marker_16": 4, "ar_marker_13": 8, "ar_marker_6": 13, "ar_marker_7": 14},
    "dawsonschess": {"ar_marker_16": 4, "ar_marker_13": 8, "ar_marker_6": 13, "ar_marker_7": 14}
}


def get_dim(game):
    """
    Returns the grid dimension for the game. Centers are defined in [0, dim] space
    and divided by this value in RobotControl.play() to normalize to [0, 1].
    """
    data = {
        "1dchess": 8,
        "allqueenschess": 1,    # already normalized
        "dawsonschess": 6,
        "dao": 1,               # already normalized
        "dodgem": 4,
        "dinododgem": 6,
        "dragonsandswans": 1,   # already normalized
        "jan": 4,
        "joust": 4,
        "ponghauki": 100,
        "beeline": 4,
        "change": 4,
        "fivefieldkono": 4,
        "foxandhounds": 4,
        "hareandhounds": 4,
        "hobaggonu": 4,
        "3spot": 3,
        "4squaretictactoe": 4,
    }
    if game not in data:
        print("get_dim: no dim defined for game '%s'" % game)
        return None
    return data[game]


def get_pickup(game):
    data = {}
    return data[game] if game in data else None


def get_capture(game):
    data = {
        "1dchess": [5.5, 2.5]
    }
    return data[game] if game in data else None


def get_piece_ARTag_frame(game, index=None):
    # Zero indexed, initial positions of ARTags
    if index is None:
        return ar_tracker[game]
    else:
        for k, v in ar_tracker[game].items():
            if v == index:
                return k


def set_piece_ARTag_frame(game, end_index, frame):
    ar_tracker[game][frame] = end_index


def process_ar_location(game, listener, start_index, end_index):
    start_frame = get_piece_ARTag_frame(game, start_index)
    set_piece_ARTag_frame(game, end_index, start_frame)
    return listener.get_pose(start_frame)