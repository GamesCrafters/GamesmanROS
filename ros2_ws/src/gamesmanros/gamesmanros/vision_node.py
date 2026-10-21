#!/usr/bin/env python3
"""
Vision node — AR tag piece localisation on a homography-rectified board.

Approach follows EECS-2025-128 section 3.4: every game piece carries a unique
ArUco tag. The tag, not the piece's appearance, is the identifier — so the same
pipeline works for any game and any piece shape without recalibration. The
thesis rejected HSV (fails when pieces share a colour, costs Pi cycles) and ML
(1000+ labelled images per piece type); both conclusions still hold.

What is different here: the thesis resolved tag poses through camera intrinsics
-> camera-to-base extrinsics -> TF tree. This node skips that entire chain. Four
ArUco markers at the board corners define a homography, and every piece tag's
centre pixel is pushed through the same matrix to land directly in board
coordinates. No intrinsics file, no extrinsic calibration, no TF. It is also
2D throughout, so there is no monocular depth term — the dominant error source
in the original 2-3 cm figure.

    camera frame
      -> detect all ArUco markers
      -> 4 corner markers (corner_ids) define homography H
      -> every other tag: centre pixel -> H -> board cell + offset
      -> vote across N frames for stability
      -> publish /vision/board_state

Piece tag IDs must not collide with corner_ids.

Parameters
  camera_index      V4L index                             (default 0)
  grid_rows/cols    board layout                          (default 4/4)
  corner_ids        4 marker IDs, order TL TR BR BL        (default [0,1,2,3])
  p1_tag_ids        tag IDs belonging to player one        (default [])
  p2_tag_ids        tag IDs belonging to player two        (default [])
  stability_frames  frames that must agree before publish  (default 5)
  debug_view        live annotated window                  (default False)
"""

from collections import Counter, deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from gamesmanros_interfaces.msg import BoardState, PieceDetection

WARP_SIZE = 600
ARUCO_DICT = cv2.aruco.DICT_4X4_50


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('grid_rows', 4)
        self.declare_parameter('grid_cols', 4)
        self.declare_parameter('corner_ids', [0, 1, 2, 3])
        self.declare_parameter('p1_tag_ids', [])
        self.declare_parameter('p2_tag_ids', [])
        self.declare_parameter('stability_frames', 5)
        self.declare_parameter('debug_view', False)

        cam            = self.get_parameter('camera_index').value
        self._rows     = self.get_parameter('grid_rows').value
        self._cols     = self.get_parameter('grid_cols').value
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

        self._cap = cv2.VideoCapture(cam)
        if not self._cap.isOpened():
            self.get_logger().error(f'cannot open camera {cam}')

        self._pub = self.create_publisher(BoardState, '/vision/board_state', 10)
        self._history = deque(maxlen=self._nvote)
        self._timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'VisionNode ready — {self._rows}x{self._cols}, corners={self._corners}, '
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

    def _locate_pieces(self, centres, H):
        """Map every non-corner tag through H into board cell coordinates."""
        cell_w = WARP_SIZE / self._cols
        cell_h = WARP_SIZE / self._rows
        out = []

        for tag_id, px in centres.items():
            if tag_id in self._corners:
                continue

            pt = cv2.perspectiveTransform(
                np.array([[px]], dtype=np.float32), H)[0][0]
            wx, wy = float(pt[0]), float(pt[1])

            col, row = wx / cell_w, wy / cell_h
            ci, ri = int(col), int(row)

            d = PieceDetection()
            d.tag_id = tag_id
            d.board_x = col
            d.board_y = row

            if 0 <= ci < self._cols and 0 <= ri < self._rows:
                d.cell_index = ri * self._cols + ci
                # Distance from the ideal centre, expressed in mm of board.
                dx = (col - (ci + 0.5)) * (150.0 / self._cols)
                dy = (row - (ri + 0.5)) * (150.0 / self._rows)
                d.offset_mm = float(np.hypot(dx, dy))
            else:
                d.cell_index = -1
                d.offset_mm = 0.0
            out.append(d)

        return out

    def _occupancy(self, pieces):
        """Per-cell string, from tag ownership rather than colour."""
        grid = ['-'] * (self._rows * self._cols)
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
        warped = cv2.warpPerspective(frame, H, (WARP_SIZE, WARP_SIZE))
        cw, ch = WARP_SIZE // self._cols, WARP_SIZE // self._rows
        for i in range(1, self._cols):
            cv2.line(warped, (i*cw, 0), (i*cw, WARP_SIZE), (0, 255, 0), 1)
        for j in range(1, self._rows):
            cv2.line(warped, (0, j*ch), (WARP_SIZE, j*ch), (0, 255, 0), 1)

        for d in pieces:
            x, y = int(d.board_x * cw), int(d.board_y * ch)
            cv2.circle(warped, (x, y), 12, (0, 140, 255), 2)
            cv2.putText(warped, str(d.tag_id), (x - 8, y - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 2)
            if d.offset_mm > 0:
                cv2.putText(warped, f'{d.offset_mm:.0f}mm', (x - 18, y + 28),
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
