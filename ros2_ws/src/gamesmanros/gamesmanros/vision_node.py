#!/usr/bin/env python3
"""
Vision node — OpenCV ArUco board detection + HSV piece classification.

Pipeline:
  1. Detect 4 corner ArUco markers (IDs 0–3) placed at board corners
  2. Compute perspective homography → warp camera frame to flat 600×600 square
  3. Divide warped image into grid cells (grid_rows × grid_cols)
  4. HSV-threshold each cell to classify as player 1, player 2, or empty
  5. Publish occupancy string to /vision/board_state

No camera intrinsics/calibration file needed — homography handles perspective.

Marker ID → board corner convention:
  0 = top-left   1 = top-right
  2 = bot-right  3 = bot-left
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

WARP_SIZE  = 600
ARUCO_DICT = cv2.aruco.DICT_4X4_50
THRESHOLD  = 0.15  # fraction of cell pixels that must match a color


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('grid_rows',    4)
        self.declare_parameter('grid_cols',    4)
        self.declare_parameter('debug_view',   False)

        # HSV bounds as flat 3-element int arrays [H, S, V]
        self.declare_parameter('p1_hsv_lower', [0,   120, 70])
        self.declare_parameter('p1_hsv_upper', [10,  255, 255])
        self.declare_parameter('p2_hsv_lower', [100, 120, 70])
        self.declare_parameter('p2_hsv_upper', [130, 255, 255])

        cam_idx        = self.get_parameter('camera_index').value
        self._rows     = self.get_parameter('grid_rows').value
        self._cols     = self.get_parameter('grid_cols').value
        self._debug    = self.get_parameter('debug_view').value
        self._p1_lower = np.array(self.get_parameter('p1_hsv_lower').value, dtype=np.uint8)
        self._p1_upper = np.array(self.get_parameter('p1_hsv_upper').value, dtype=np.uint8)
        self._p2_lower = np.array(self.get_parameter('p2_hsv_lower').value, dtype=np.uint8)
        self._p2_upper = np.array(self.get_parameter('p2_hsv_upper').value, dtype=np.uint8)

        aruco_dict     = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        params         = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, params)

        self._cap = cv2.VideoCapture(cam_idx)
        if not self._cap.isOpened():
            self.get_logger().error(f'Cannot open camera {cam_idx}')

        self._pub   = self.create_publisher(String, '/vision/board_state', 10)
        self._timer = self.create_timer(0.1, self._process_frame)  # 10 Hz

        self.get_logger().info(
            f'VisionNode ready — {self._rows}x{self._cols} grid, camera {cam_idx}')

    # ------------------------------------------------------------------

    def _process_frame(self):
        ret, frame = self._cap.read()
        if not ret:
            return

        corners, ids, _ = self._detector.detectMarkers(frame)

        if ids is None:
            return

        # Build id → marker center pixel map
        id_map = {}
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in (0, 1, 2, 3):
                id_map[int(marker_id)] = corners[i][0].mean(axis=0)

        if len(id_map) < 4:
            return  # need all 4 corner markers

        # Perspective warp: marker centers → WARP_SIZE square
        # ID order: TL, TR, BR, BL matches dst corners
        src = np.array([id_map[0], id_map[1], id_map[2], id_map[3]], dtype=np.float32)
        dst = np.array([
            [0,         0        ],
            [WARP_SIZE, 0        ],
            [WARP_SIZE, WARP_SIZE],
            [0,         WARP_SIZE],
        ], dtype=np.float32)

        H      = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(frame, H, (WARP_SIZE, WARP_SIZE))

        occupancy = self._classify_cells(warped)

        msg      = String()
        msg.data = ''.join(occupancy)
        self._pub.publish(msg)

        if self._debug:
            self._draw_debug(warped, occupancy)

    def _classify_cells(self, warped: np.ndarray) -> list:
        hsv     = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        cell_h  = WARP_SIZE // self._rows
        cell_w  = WARP_SIZE // self._cols
        cell_px = cell_h * cell_w

        occupancy = []
        for row in range(self._rows):
            for col in range(self._cols):
                y0   = row * cell_h
                x0   = col * cell_w
                cell = hsv[y0:y0 + cell_h, x0:x0 + cell_w]

                n1 = cv2.countNonZero(cv2.inRange(cell, self._p1_lower, self._p1_upper))
                n2 = cv2.countNonZero(cv2.inRange(cell, self._p2_lower, self._p2_upper))

                if   n1 / cell_px > THRESHOLD:
                    occupancy.append('1')
                elif n2 / cell_px > THRESHOLD:
                    occupancy.append('2')
                else:
                    occupancy.append('-')

        return occupancy

    def _draw_debug(self, warped: np.ndarray, occupancy: list):
        cell_h = WARP_SIZE // self._rows
        cell_w = WARP_SIZE // self._cols

        for i in range(1, self._rows):
            cv2.line(warped, (0, i * cell_h), (WARP_SIZE, i * cell_h), (0, 255, 0), 1)
        for j in range(1, self._cols):
            cv2.line(warped, (j * cell_w, 0), (j * cell_w, WARP_SIZE), (0, 255, 0), 1)

        for idx, label in enumerate(occupancy):
            row, col = divmod(idx, self._cols)
            cx    = col * cell_w + cell_w // 2
            cy    = row * cell_h + cell_h // 2
            color = (255, 80, 80) if label == '1' else (80, 80, 255) if label == '2' else (160, 160, 160)
            cv2.putText(warped, label, (cx - 8, cy + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

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
    finally:
        node.destroy_node()
        rclpy.shutdown()
