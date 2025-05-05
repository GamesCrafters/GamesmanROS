"""
Get Centers for the Robot

Objective is to standardize centers for the games by ensuring that the maximum values 
for the centers are (1,1) and the minimum is (0,0).

NOTE: Leave the coordinates in SVG coordinate space

{id: [list of centers]}
id: Must Match GameID from UWAPI init functions
[list of centers]: Similar to UWAPI imageautogui data, provide a list of centers with a max ratio of 1:1
"""

def get_centers():
    return {
        "1dchess": [[0.5 + i, 0.5] for i in range(8)],
        "allqueenschess": [[(i % 5 + 0.5) / 5, (i // 5 + 0.5) / 5] for i in range(25)],
        "dawsonschess": [[0.5,1.5],[1.5,1.5],[2.5,1.5],[3.5,1.5],[4.5,2.5],[5.5,2.5]],
        "dao": [[(i % 4 + 0.5) / 4, (i // 4 + 0.5) / 4] for i in range(16)],
        "dodgem": [[(i % 4 + 0.5), (i // 4 + 0.5)] for i in range(16)],
        "dinododgem": [[i // 5 + 0.5, 5.5 - (i % 5)] for i in range(25)] + [
                    [1.5, 0.5], [2.5, 0.5], [3.5, 0.5], [4.5, 0.5], 
                    [5.5, 1.5], [5.5, 2.5], [5.5, 3.5], [5.5, 4.5]
                ],
        "dragonsandswans": [[(i % 4 * 10 + 5) / 35, (i // 4 * 10 + 5) / 46] for i in range(16)] + [[28.7/35, 43/46], [30.2/35, 43/46], [28.7/35, 46/46], [30.2/35, 46/46]],
        "jan": [[((i % 4)), ((i // 4))] for i in range(16)],
        "joust": [[0.5,0.5],[1.5,0.5],[2.5,0.5],[3.5,0.5],[0.5,1.5],[1.5,1.5],[2.5,1.5],[3.5,1.5],[0.5,2.5],[1.5,2.5],[2.5,2.5],[3.5,2.5],[0.5,3.5],[1.5,3.5],[2.5,3.5],[3.5,3.5]]
    }

def get_dim():
    return {
        "1dchess": 8,
        "allqueenschess": 5,
        "dawsonschess": 5,
        "dao": 4,
        "dodgem": 3,
        "jan": 3,
        "joust": 4
    }

def get_pickup():
    return [None, None]

def get_capture(game):
    if game == "1dchess":
        return [5.5, 2.5]
    else:
        return [None, None]

games = {"dodgem" : {"ar_marker_16" : 4, "ar_marker_13" : 8, "ar_marker_6" : 13, "ar_marker_7" : 14}}

def get_piece_ARTag_frame(game, index=None):
    # Zero indexed, initial positions of ARTags
    if index == None:
        return games[game]
    else:
        for k,v in games[game].items():
            if v == index:
                return k

def set_piece_ARTag_frame(game, end_index, frame):
    games[game][frame] = end_index