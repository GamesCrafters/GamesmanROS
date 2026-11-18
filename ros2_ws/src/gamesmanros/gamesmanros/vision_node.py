"""
Vision node — AR tag piece localisation on a homography-rectified board.

Follows EECS-2025-128 section 3.4: every piece carries a unique ArUco tag, so
the tag (not the piece's appearance) is the identifier. That is what lets one
pipeline serve every game without recalibration. The thesis rejected HSV (fails
on same-coloured pieces, costs Pi cycles) and ML (1000+ labelled images per
piece type); both conclusions still hold.

Two things differ from the thesis:

1. It resolved tag poses through camera intrinsics -> extrinsics -> TF tree.
   Four ArUco markers at the board corners define a homography instead, so a
   tag's centre pixel maps straight into board space. No intrinsics, no
   calibration step, no TF. It is also 2D throughout, so there is no monocular
   depth term -- the dominant error source in the original 2-3 cm figure.

2. Cells are NOT assumed to lie on a grid. Cell positions come from
   centers.get_centers(game, variant), which fetches the real AutoGUI layout
   from UWAPI and fits it into a unit square. A detected tag is assigned to the
   NEAREST cell centre. That means irregular boards work for free:

       dodgem      16 cells, 4x4 grid
       1dchess      8 cells, 8x1 strip
       ponghauki    5 cells in an X -- a graph, not a grid at all

   The old rows x cols division could not represent the last two.

    camera frame
      -> detect all ArUco markers
      -> 4 corner markers (corner_ids) define homography H
      -> every other tag: centre pixel -> H -> unit square -> nearest cell
      -> vote across N frames for stability
      -> publish /vision/board_state

Piece tag IDs must not collide with corner_ids.

Parameters
  camera_index      V4L index                               (default 0)
  game / variant    which board layout to fetch from UWAPI  (dodgem/regular)
  corner_ids        4 marker IDs, order TL TR BR BL          (default [0,1,2,3])
  p1_tag_ids        tag IDs belonging to player one          (default [])
  p2_tag_ids        tag IDs belonging to player two          (default [])
  snap_radius       max distance (unit-square units) from a
                    cell centre before a tag counts as
                    off-board                                (default 0.12)
  stability_frames  frames that must agree before publish    (default 5)
  debug_view        live annotated window                    (default False)
"""

from collections import Counter, deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from gamesmanros_interfaces.msg import BoardState, PieceDetection
from .centers import GeometryError, get_centers

WARP_SIZE = 600
BOARD_SIZE_MM = 150.0   # matches calibration.BOARD_WIDTH_MM
ARUCO_DICT = cv2.aruco.DICT_4X4_50


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('game', 'dodgem')
        self.declare_parameter('variant', 'regular')
        self.declare_parameter('snap_radius', 0.12)
        self.declare_parameter('corner_ids', [0, 1, 2, 3])
        self.declare_parameter('p1_tag_ids', [])
        self.declare_parameter('p2_tag_ids', [])
        self.declare_parameter('stability_frames', 5)
        self.declare_parameter('debug_view', False)

        cam            = self.get_parameter('camera_index').value
        self._game     = self.get_parameter('game').value
        self._variant  = self.get_parameter('variant').value
        self._snap     = float(self.get_parameter('snap_radius').value)
        self._corners  = list(self.get_parameter('corner_ids').value)
        self._p1       = set(self.get_parameter('p1_tag_ids').value)
        self._p2       = set(self.get_parameter('p2_tag_ids').value)
        self._nvote    = max(1, self.get_parameter('stability_frames').value)
        self._debug    = self.get_parameter('debug_view').value

        if len(self._corners) != 4:
            self.get_logger().error('corner_ids must list exactly 4 marker IDs')
        overlap = (self._p1 | self._p2) & set(self._corners)
        if overlap:
            self.get_logger().error(
                f'piece tags {sorted(overlap)} collide with corner_ids — '
                f'the board corners will be read as pieces')

        self._detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(ARUCO_DICT),
            cv2.aruco.DetectorParameters())

        # Real cell positions for this board, in a unit square. Grid games and
        # graph games (ponghauki) are both just a list of points here.
        self._cells = self._load_cells()

        self._cap = cv2.VideoCapture(cam)
        if not self._cap.isOpened():
            self.get_logger().error(f'cannot open camera {cam}')

        self._pub = self.create_publisher(BoardState, '/vision/board_state', 10)
        self._history = deque(maxlen=self._nvote)
        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'VisionNode ready — {self._game}/{self._variant}, corners={self._corners}, '
            f'{len(self._p1)}+{len(self._p2)} piece tags, voting over {self._nvote} frames')

    # ------------------------------------------------------------------

    def _tick(self):
        ok, frame = self._cap.read()
        if not ok:
            return

        corners, ids, _ = self._detector.detectMarkers(frame)
        if ids is None:
            self._publish([], '', located=False, voted=0)
            return

        centres = {int(i): c[0].mean(axis=0)
                   for i, c in zip(ids.flatten(), corners)}

        H = self._homography(centres)
        if H is None:
            self._publish([], '', located=False, voted=0)
            return

        pieces = self._locate_pieces(centres, H)
        occupancy = self._occupancy(pieces)

        # Vote: only publish a state once the same occupancy string has been
        # seen stability_frames times running. A single bad frame, a hand
        # passing over the board, or a half-occluded tag cannot move the state.
        self._history.append(occupancy)
        counts = Counter(self._history)
        best, n = counts.most_common(1)[0]
        if n >= self._nvote and best == occupancy:
            self._publish(pieces, occupancy, located=True, voted=n)

        if self._debug:
            self._draw(frame, H, pieces, occupancy)

    def _homography(self, centres):
        """Perspective transform from the 4 corner markers, or None if unseen."""
        if not all(c in centres for c in self._corners):
            return None
        src = np.array([centres[c] for c in self._corners], dtype=np.float32)
        dst = np.array([[0, 0], [WARP_SIZE, 0],
                        [WARP_SIZE, WARP_SIZE], [0, WARP_SIZE]], dtype=np.float32)
        return cv2.getPerspectiveTransform(src, dst)

    def _load_cells(self):
        """Cell centres in the unit square, index-aligned with UWAPI positions.

        Entries that UWAPI marks as non-playable come back as None and are kept
        as None so that cell_index stays aligned with the position string.
        """
        try:
            cells = get_centers(self._game, self._variant)
        except GeometryError as e:
            self.get_logger().error(
                f'could not load geometry for {self._game}/{self._variant}: {e}. '
                f'Pieces will be reported with cell_index = -1.')
            return []
        usable = sum(1 for c in cells if c is not None)
        self.get_logger().info(
            f'{self._game}/{self._variant}: {len(cells)} cells, {usable} playable')
        return cells

    def _nearest_cell(self, u, v):
        """(index, distance) of the closest playable cell, or (-1, 0.0)."""
        best_i, best_d = -1, None
        for i, c in enumerate(self._cells):
            if c is None:
                continue
            d = np.hypot(u - c[0], v - c[1])
            if best_d is None or d < best_d:
                best_i, best_d = i, d
        if best_d is None or best_d > self._snap:
            return -1, 0.0
        return best_i, float(best_d)

    def _locate_pieces(self, centres, H):
        """Map every non-corner tag through H, then snap it to the nearest cell."""
        out = []
        for tag_id, px in centres.items():
            if tag_id in self._corners:
                continue

            pt = cv2.perspectiveTransform(
                np.array([[px]], dtype=np.float32), H)[0][0]
            # Warped pixels -> unit square, which is the space centers.py uses.
            u, v = float(pt[0]) / WARP_SIZE, float(pt[1]) / WARP_SIZE

            idx, dist = self._nearest_cell(u, v)

            d = PieceDetection()
            d.tag_id = tag_id
            d.board_x = u
            d.board_y = v
            d.cell_index = idx
            # Unit-square distance -> mm on the physical board.
            d.offset_mm = float(dist * BOARD_SIZE_MM)
            out.append(d)

        return out

    def _occupancy(self, pieces):
        """Per-cell string, from tag ownership rather than colour."""
        grid = ['-'] * len(self._cells)
        for d in pieces:
            if d.cell_index < 0:
                continue
            if d.tag_id in self._p1:
                grid[d.cell_index] = '1'
            elif d.tag_id in self._p2:
                grid[d.cell_index] = '2'
            else:
                grid[d.cell_index] = '?'   # tag seen but unassigned
        return ''.join(grid)

    def _publish(self, pieces, occupancy, located, voted):
        msg = BoardState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'board'
        msg.occupancy = occupancy
        msg.pieces = pieces
        msg.board_located = located
        msg.frames_voted = voted
        self._pub.publish(msg)

    def _draw(self, frame, H, pieces, occupancy):
        """Rectified board with the real cell layout drawn, not a grid."""
        warped = cv2.warpPerspective(frame, H, (WARP_SIZE, WARP_SIZE))

        # Actual cell centres and the snap radius around each.
        for i, c in enumerate(self._cells):
            if c is None:
                continue
            cx, cy = int(c[0] * WARP_SIZE), int(c[1] * WARP_SIZE)
            cv2.circle(warped, (cx, cy), int(self._snap * WARP_SIZE), (60, 90, 60), 1)
            cv2.circle(warped, (cx, cy), 3, (0, 255, 0), -1)
            cv2.putText(warped, str(i), (cx + 7, cy - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1)

        # Detected tags, with a line to the cell they snapped to.
        for d in pieces:
            x, y = int(d.board_x * WARP_SIZE), int(d.board_y * WARP_SIZE)
            hit = 0 <= d.cell_index < len(self._cells)
            colour = (0, 140, 255) if hit else (0, 0, 255)
            cv2.circle(warped, (x, y), 11, colour, 2)
            cv2.putText(warped, str(d.tag_id), (x - 8, y - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
            if hit:
                c = self._cells[d.cell_index]
                cv2.line(warped, (x, y),
                         (int(c[0] * WARP_SIZE), int(c[1] * WARP_SIZE)), colour, 1)
                cv2.putText(warped, f'{d.offset_mm:.0f}mm', (x - 18, y + 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        cv2.putText(warped, occupancy, (10, WARP_SIZE - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow('VisionNode', warped)
        cv2.waitKey(1)

    def destroy_node(self):
        self._cap.release()
        if self._debug:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
