import numpy as np
from low_level_controller import *
import time
from centers import get_piece_ARTag_frame, set_piece_ARTag_frame, get_dim
from findPiece import PieceFinder

########################################################
def getType(gameId):
    types_of_games = {"Type1": ["dawsonschess"],
                      "Type4": ["3spot", "allqueenschess", "beeline", "change", "dao", "fivefieldkono", 
                                "foxandhounds", "hareandhounds", "jan", "joust", "hobaggonu"],
                      "Type6": ["dinododgem", "dodgem"],
                      "Type7": ["1dchess"]}
    types = {"Type1" : Type1, "Type4" : Type4, "Type6" : Type6, "Type7" : Type7}
    
    for gameType in types_of_games:
        if gameId in types_of_games[gameType]:
            return types[gameType]
    
    print("Error in RobotControl GameType!")
    return None

########################################################

"""
Place
"""
class Type1:
    def __init__(self, game, centers, pickup=None, capture=None):
        self.centers = centers
        self.counter = 0
        self.pickup = [[0.5,3.5], [1.5,3.5], [2.5,3.5]],
        self.control = RobotControl()
        self.game = game

    def processMove(self, move, positions=None):
        move_string_split = move.split('_')
        start_index = self.counter
        self.counter += 1
        end_index = int(move_string_split[2])

        coords = [self.pickup[0][start_index], self.centers[end_index]]
        self.playMove(coords)
        return coords
    
    def playMove(self, coords):
        before, after = coords
        self.control.play(before, after)

"""
Captures
"""
class Type2:
    def __init__(self, centers, pickup, capture):
        self.centers = centers
        self.pickup = pickup
        self.capture = capture

    def processMove(self, move, positions=None):
        move_string_split = move.split('_')
        start_index = int(move_string_split[1])
        end_index = int(move_string_split[2])

        start_cord = self.centers[start_index]
        end_cord = self.centers[end_index]

        start_position, end_position = positions
        if start_position[end_index] != end_position[end_index] and start_position[end_index] != '-':
            return [end_cord, self.capture, start_cord, end_cord]
        return [start_cord, end_cord]
    
"""
Removal
"""
class Type3:
    def __init__(self, centers):
        self.centers = centers

    def processMove(self, move, positions=None):
        # move_string_split = move.split('_')
        # start_index = int(move_string_split[1])
        # end_index = int(move_string_split[2])
        return None
    
"""
Re-Arranger
"""
class Type4:
    def __init__(self, game, centers, pickup=None, capture=None):
        self.centers = centers
        self.control = RobotControl(dim=get_dim()[game])
        self.game = game

    def processMove(self, move, positions=None):
        move_string_split = move.split('_')
        start_index = int(move_string_split[1])
        end_index = int(move_string_split[2])

        # start_frame = get_piece_ARTag_frame(self.game, start_index)
        # set_piece_ARTag_frame(self.game, end_index, start_frame)

        start_cord = self.centers[start_index]
        end_cord = self.centers[end_index]
        coords = [start_cord, end_cord]
        self.playMove(coords)
        return coords

    def playMove(self, coords):
        before, after = coords
        self.control.play(before, after)

"""
Place + Re-Arranger
"""
class Type5:
    def __init__(self, centers):
        self.centers = centers

    def processMove(self, move, positions=None):
        # move_string_split = move.split('_')
        # start_index = int(move_string_split[1])
        # end_index = int(move_string_split[2])
        return None

"""
Re-Arranger + Removal
"""
class Type6:
    def __init__(self, game, centers, pickup=None, capture=None):
        self.centers = centers
        self.control = RobotControl(dim=get_dim()[game])
        self.game = game

    def processMove(self, move, positions=None):
        move_string_split = move.split('_')
        start_index = int(move_string_split[1])
        end_index = int(move_string_split[2])

        # start_frame = get_piece_ARTag_frame(self.game, start_index)
        # set_piece_ARTag_frame(self.game, end_index, start_frame)

        start_cord = self.centers[start_index]
        end_cord = self.centers[end_index]
        coords = [start_cord, end_cord]
        self.playMove(coords)
        return coords

    def playMove(self, coords):
        before, after = coords
        self.control.play(before, after)


"""
Re-Arranger + Capture
"""
class Type7:
    def __init__(self, game, centers, pickup=None, capture=None):
        self.centers = centers
        self.pickup = pickup
        self.capture = capture
        self.control = RobotControl(dim=get_dim()[game])

    def processMove(self, move, positions=None):
        move_string_split = move.split('_')
        start_index = int(move_string_split[1])
        end_index = int(move_string_split[2])

        start_cord = self.centers[start_index]
        end_cord = self.centers[end_index]

        start_position, end_position = positions
        if start_position[end_index] != end_position[end_index] and start_position[end_index] != '-':
            coords = [end_cord, self.capture, start_cord, end_cord]
            self.playMove(coords)
            return coords
        
        coords = [start_cord, end_cord]
        self.playMove(coords)
        return coords

    def playMove(self, coords):
        if len(coords) == 2:
            before, after = coords
            self.control.play(before, after)
        else:
            before1, after1, before2, after2 = coords
            self.control.play(before1, after1)
            self.control.play(before2, after2)

"""
Place + Re-Arranger + Removal
"""
class Type8:
    def __init__(self, centers):
        self.centers = centers

    def processMove(self, move, positions=None):
        # move_string_split = move.split('_')
        # start_index = int(move_string_split[1])
        # end_index = int(move_string_split[2])
        return None

"""
Place + Re-Arranger + Capture
"""
class Type9:
    def __init__(self, centers):
        self.centers = centers

    def processMove(self, move, positions=None):
        # move_string_split = move.split('_')
        # start_index = int(move_string_split[1])
        # end_index = int(move_string_split[2])
        return None
    
"""
Re-Arranger + Capture + Removal
"""
class Type10:
    def __init__(self, centers):
        self.centers = centers

    def processMove(self, move, positions=None):
        # move_string_split = move.split('_')
        # start_index = int(move_string_split[1])
        # end_index = int(move_string_split[2])
        return None

"""
Place + Re-Arranger + Capture + Removal
"""
class Type11:
    def __init__(self, centers):
        self.centers = centers

    def processMove(self, move, positions=None):
        # move_string_split = move.split('_')
        # start_index = int(move_string_split[1])
        # end_index = int(move_string_split[2])
        return None

###################################################################
###################################################################


class RobotControl:
    def __init__(self, board_size=150, dim=3, y_offset=100, pickup_z=135, lift_z=185):
        self.board_size = board_size
        self.dim = dim
        self.scaling = self.board_size/(self.dim)
        self.x_offset = (self.board_size/2) + 50
        self.y_offset = y_offset
        self.pickup_z = pickup_z
        self.lift_z = lift_z / 1000

    def svg_to_real(self, svg_coord):
        T = np.array([[1, 0, 0],
                    [0, -1, self.dim+1],
                    [0, 0, 1]])
        
        # T = np.array([[1, 0, 0],
        #         [0, -1, 0],
        #         [0, 0, 1]])

        coord = np.array([svg_coord[0], svg_coord[1], 1])

        real_coord = np.dot(T, coord.T)
        real_coord[1] = abs(real_coord[1])
        return [real_coord[0], real_coord[1]]

    #gripper: Open 0, Close 1
    def play(self, before, after):
        before = self.svg_to_real(before)
        x = (before[0] * self.scaling) - self.x_offset
        y = (before[1] * self.scaling) + self.y_offset
        z = self.pickup_z

        x = x / 1000
        y = y / 1000
        z = z / 1000

        after = self.svg_to_real(after)
        after_x = (after[0] * self.scaling) - self.x_offset
        after_y = (after[1] * self.scaling) + self.y_offset
        after_z = self.pickup_z

        after_x = after_x / 1000
        after_y = after_y / 1000
        after_z = after_z / 1000

        print("Before: ", (x, y, z), " | ", "After: ", (after_x, after_y, after_z))

        flag = True

        gripper_status("open")
        time.sleep(0.5)

        if flag:
            flag = plan_to_xyz(x, y, self.lift_z)
            time.sleep(1)
        if flag:
            flag = plan_to_xyz(x, y, z)
            time.sleep(1)
            flag = plan_to_xyz(x, y, z)

        if flag:
            gripper_status("close")
            time.sleep(0.5)
            gripper_status("close")
        
        if flag:
            flag = plan_to_xyz(x, y, self.lift_z)
            time.sleep(1)
        if flag:
            flag = plan_to_xyz(after_x, after_y, self.lift_z)
            time.sleep(1)
        if flag:
            flag = plan_to_xyz(after_x, after_y, after_z)
            time.sleep(1)
            flag = plan_to_xyz(after_x, after_y, after_z)

        if flag:
            gripper_status("open")
            time.sleep(0.5)
            gripper_status("open")

        if flag:
            flag = plan_to_xyz(after_x, after_y, self.lift_z)
            time.sleep(1)
            
        return flag