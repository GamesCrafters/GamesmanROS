import numpy as np
from low_level_controller import *
import time
from centers import *
import threading
import rospy
from artag_listener import ARTagListener

########################################################
def getType(gameId):
    types_of_games = {
        "Type1": ["dawsonschess", "4squaretictactoe"],
        "Type4": ["3spot", "allqueenschess", "beeline", "change", "dao", "fivefieldkono",
                  "foxandhounds", "hareandhounds", "jan", "joust", "hobaggonu", "ponghauki"],
        "Type6": ["dinododgem", "dodgem"],
        "Type7": ["1dchess"]
    }
    types = {"Type1": Type1, "Type4": Type4, "Type6": Type6, "Type7": Type7}

    for gameType in types_of_games:
        if gameId in types_of_games[gameType]:
            return types[gameType]

    print("Error in RobotControl GameType! No type found for game:", gameId)
    return None

########################################################

class BaseType:
    def __init__(self, game, svg_space=[1, 1], vision=False):
        self.game = game
        self.vision = vision
        self.svg_space = svg_space
        self.centers = get_centers(game)
        self.pickup = get_pickup(game)
        self.capture = get_capture(game)

        self.control = RobotControl(svg_space=self.svg_space)

        if self.vision:
            self.listener = ARTagListener()

            if not rospy.core.is_initialized():
                rospy.init_node("robot_control_node", anonymous=True)

            self.spin_thread = threading.Thread(target=rospy.spin, daemon=True)
            self.spin_thread.start()

"""
Place
"""
class Type1(BaseType):
    def __init__(self, game, svg_space=[1, 1], vision=False):
        super().__init__(game, svg_space, vision)
        self.counter = 0

    def processMove(self, move, positions=None):
        move_string_split = move.split('_')
        start_index = self.counter
        self.counter += 1
        end_index = int(move_string_split[2])

        if self.pickup is None:
            rospy.logwarn("Type1: no pickup locations defined for game '%s'" % self.game)
            return None

        coords = [self.pickup[0][start_index], self.centers[end_index]]
        self.playMove(coords)
        return coords

    def playMove(self, coords):
        before, after = coords
        self.control.play(before, after)

"""
Captures
"""
class Type2(BaseType):
    def __init__(self, game, svg_space=[1, 1], vision=False):
        super().__init__(game, svg_space, vision)

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
class Type3(BaseType):
    def __init__(self, game, svg_space=[1, 1], vision=False):
        super().__init__(game, svg_space, vision)

    def processMove(self, move, positions=None):
        return None

"""
Re-Arranger
"""
class Type4(BaseType):
    def __init__(self, game, svg_space=[1, 1], vision=False):
        super().__init__(game, svg_space, vision)

    def processMove(self, move, positions=None):
        move_string_split = move.split('_')
        start_index = int(move_string_split[1])
        end_index = int(move_string_split[2])

        if self.vision:
            start_coord = process_ar_location(self.game, self.listener, start_index, end_index)
            end_coord = self.centers[end_index]
        else:
            start_coord, end_coord = (self.centers[start_index], self.centers[end_index])

        coords = [start_coord, end_coord]
        self.playMove(coords)
        return coords

    def playMove(self, coords):
        before, after = coords
        self.control.play(before, after)

"""
Place + Re-Arranger
"""
class Type5(BaseType):
    def __init__(self, game, svg_space=[1, 1], vision=False):
        super().__init__(game, svg_space, vision)

    def processMove(self, move, positions=None):
        return None

"""
Re-Arranger + Removal
"""
class Type6(BaseType):
    def __init__(self, game, svg_space=[1, 1], vision=False):
        super().__init__(game, svg_space, vision)

    def processMove(self, move, positions=None):
        move_string_split = move.split('_')
        start_index = int(move_string_split[1])
        end_index = int(move_string_split[2])

        if self.vision:
            start_coord = process_ar_location(self.game, self.listener, start_index, end_index)
            end_coord = self.centers[end_index]
        else:
            start_coord, end_coord = (self.centers[start_index], self.centers[end_index])

        coords = [start_coord, end_coord]
        self.playMove(coords)
        return coords

    def playMove(self, coords):
        before, after = coords
        self.control.play(before, after)

"""
Re-Arranger + Capture
"""
class Type7(BaseType):
    def __init__(self, game, svg_space=[1, 1], vision=False):
        super().__init__(game, svg_space, vision)

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
class Type8(BaseType):
    def __init__(self, game, svg_space=[1, 1], vision=False):
        super().__init__(game, svg_space, vision)

    def processMove(self, move, positions=None):
        return None

"""
Place + Re-Arranger + Capture
"""
class Type9(BaseType):
    def __init__(self, game, svg_space=[1, 1], vision=False):
        super().__init__(game, svg_space, vision)

    def processMove(self, move, positions=None):
        return None

"""
Re-Arranger + Capture + Removal
"""
class Type10(BaseType):
    def __init__(self, game, svg_space=[1, 1], vision=False):
        super().__init__(game, svg_space, vision)

    def processMove(self, move, positions=None):
        return None

"""
Place + Re-Arranger + Capture + Removal
"""
class Type11(BaseType):
    def __init__(self, game, svg_space=[1, 1], vision=False):
        super().__init__(game, svg_space, vision)

    def processMove(self, move, positions=None):
        return None

###################################################################
###################################################################


class RobotControl:
    def __init__(self, board_size=150, svg_space=[1, 1], y_offset=145, pickup_z=140, lift_z=165):
        self.board_size = board_size
        self.svg_space = svg_space
        self.x_offset = self.board_size / 2
        self.y_offset = y_offset
        self.pickup_z = pickup_z
        # lift_z lowered from 185 to 165 mm.
        # 185 mm caused NO_IK_SOLUTION at edge board positions (x ~ +/-0.0375)
        # because the arm cannot reach that height at the extremes of the 150x150 grid.
        # 165 mm clears the 3D printed pieces while staying within the reachable envelope.
        # If pieces are taller than ~25 mm, increase this cautiously and re-test edge positions.
        self.lift_z = lift_z / 1000

    def svg_to_real(self, svg_coord):
        T = np.array([[1, 0, 0],
                      [0, -1, 1],
                      [0, 0, 1]])
        coord = np.array([svg_coord[0], svg_coord[1], 1])
        real_coord = np.dot(T, coord.T)
        real_coord[1] = abs(real_coord[1])
        print("SVG_TO_REAL: ", svg_coord, real_coord)
        return [real_coord[0], real_coord[1]]

    def play(self, before, after):
        print(before, self.svg_space)

        # Normalize SVG coords to [0, 1] using svg_space, then convert to real world
        before = [before[0] / self.svg_space[0], before[1] / self.svg_space[1]]
        before = self.svg_to_real(before)
        x = (before[0] * self.board_size) - self.x_offset
        y = (before[1] * self.board_size) + self.y_offset
        z = self.pickup_z

        x = x / 1000
        y = y / 1000
        z = z / 1000

        after = [after[0] / self.svg_space[0], after[1] / self.svg_space[1]]
        after = self.svg_to_real(after)
        after_x = (after[0] * self.board_size) - self.x_offset
        after_y = (after[1] * self.board_size) + self.y_offset
        after_z = self.pickup_z

        after_x = after_x / 1000
        after_y = after_y / 1000
        after_z = after_z / 1000

        print("Before: ", (x, y, z), " | ", "After: ", (after_x, after_y, after_z))

        gripper_status("open")
        time.sleep(0.5)

        flag = plan_to_xyz(x, y, self.lift_z)
        if not flag:
            rospy.logwarn("Lift move failed at before position — robot may not clear pieces!")
        time.sleep(1)

        flag = plan_to_xyz(x, y, z)
        time.sleep(1)
        flag = plan_to_xyz(x, y, z)

        gripper_status("close")
        time.sleep(0.5)
        gripper_status("close")

        flag = plan_to_xyz(x, y, self.lift_z)
        if not flag:
            rospy.logwarn("Lift move failed after pickup — robot may drag piece!")
        time.sleep(1)

        flag = plan_to_xyz(after_x, after_y, self.lift_z)
        time.sleep(1)
        flag = plan_to_xyz(after_x, after_y, after_z)
        time.sleep(1)
        flag = plan_to_xyz(after_x, after_y, after_z)

        gripper_status("open")
        time.sleep(0.5)
        gripper_status("open")

        flag = plan_to_xyz(after_x, after_y, self.lift_z)
        time.sleep(1)

        return flag