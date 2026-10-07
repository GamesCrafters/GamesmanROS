import cv2
import numpy as np
from ultralytics import YOLO
from collections import deque, Counter
import time

class RoboticArmPieceTracker:
    """Advanced tracker with process of elimination for robotic arm applications"""
    def __init__(self, max_distance=100, memory_frames=180, reidentification_distance=250):
        self.next_id = {'black piece': 1, 'white piece': 1}
        self.all_pieces_ever_seen = {}
        self.active_pieces = {}
        self.missing_pieces = {}
        self.max_distance = max_distance
        self.memory_frames = memory_frames
        self.reidentification_distance = reidentification_distance
        
        self.scene_memory = None
        self.frames_since_full_loss = 0
        self.camera_moving = False
        
    class TrackData:
        """Store tracking information for a piece"""
        def __init__(self, piece_id, color, position, bbox, frame_num):
            self.id = piece_id
            self.color = color
            self.position = position
            self.bbox = bbox
            self.history = deque(maxlen=100)
            self.history.append((frame_num, position))
            self.last_seen_frame = frame_num
            self.frames_missing = 0
            self.total_frames_tracked = 1
            self.velocity = (0, 0)
            self.first_seen_frame = frame_num
            
        def update(self, position, bbox, frame_num):
            if len(self.history) > 0:
                last_frame, last_pos = self.history[-1]
                dt = frame_num - last_frame
                if dt > 0:
                    self.velocity = (
                        (position[0] - last_pos[0]) / dt,
                        (position[1] - last_pos[1]) / dt
                    )
            
            self.position = position
            self.bbox = bbox
            self.history.append((frame_num, position))
            self.last_seen_frame = frame_num
            self.frames_missing = 0
            self.total_frames_tracked += 1
            
        def predict_position(self, current_frame):
            if len(self.history) < 2:
                return self.position
            
            frames_since = current_frame - self.last_seen_frame
            predicted_x = self.position[0] + self.velocity[0] * frames_since
            predicted_y = self.position[1] + self.velocity[1] * frames_since
            
            return (int(predicted_x), int(predicted_y))
    
    def get_missing_ids(self, color):
        missing = []
        for pid, track in self.all_pieces_ever_seen.items():
            if track.color == color and pid not in self.active_pieces:
                missing.append(pid)
        return missing
    
    def update(self, detections, frame_num):
        if len(detections) == 0 and len(self.active_pieces) > 2:
            if not self.camera_moving:
                self.scene_memory = dict(self.active_pieces)
                self.camera_moving = True
                self.frames_since_full_loss = 0
                
            self.frames_since_full_loss += 1
        elif len(detections) > 2 and self.camera_moving:
            self.camera_moving = False
          
        
        to_move_to_missing = []
        for pid, track in self.active_pieces.items():
            track.frames_missing += 1
            if track.frames_missing > 15:
                to_move_to_missing.append(pid)
        
        for pid in to_move_to_missing:
            self.missing_pieces[pid] = self.active_pieces[pid]
            del self.active_pieces[pid]
        
        to_remove = []
        for pid, track in self.missing_pieces.items():
            if track.frames_missing > self.memory_frames:
                to_remove.append(pid)
        for pid in to_remove:
            del self.missing_pieces[pid]
        
        matched_tracks = set()
        matched_detections = set()
        tracked_results = []
        

        for i, (color, cx, cy, bbox, dist) in enumerate(detections):
            best_match = None
            min_dist = float('inf')
            
            for pid, track in self.active_pieces.items():
                if track.color == color and pid not in matched_tracks:
                    predicted_pos = track.predict_position(frame_num)
                    d = np.sqrt((cx - predicted_pos[0])**2 + (cy - predicted_pos[1])**2)
                    
                    if d < self.max_distance and d < min_dist:
                        min_dist = d
                        best_match = pid
            
            if best_match:
                track = self.active_pieces[best_match]
                track.update((cx, cy), bbox, frame_num)
                matched_tracks.add(best_match)
                matched_detections.add(i)
                tracked_results.append((color, best_match, cx, cy, bbox, dist, 'active', 1.0))
        
    
        unmatched_detections = [(i, d) for i, d in enumerate(detections) if i not in matched_detections]
        
        for det_idx, (color, cx, cy, bbox, dist) in unmatched_detections:
            missing_ids = self.get_missing_ids(color)
            
            if len(missing_ids) == 1:
                piece_id = missing_ids[0]
                track = self.missing_pieces[piece_id]
                track.update((cx, cy), bbox, frame_num)
                self.active_pieces[piece_id] = track
                del self.missing_pieces[piece_id]
                matched_tracks.add(piece_id)
                tracked_results.append((color, piece_id, cx, cy, bbox, dist, 'eliminated', 1.0))
                print(f"✓ Re-identified {piece_id} by process of elimination!")
                
            elif len(missing_ids) > 1:
                best_match = None
                min_dist = float('inf')
                confidence = 0.0
                
                for pid in missing_ids:
                    track = self.missing_pieces[pid]
                    predicted_pos = track.predict_position(frame_num)
                    d = np.sqrt((cx - predicted_pos[0])**2 + (cy - predicted_pos[1])**2)
                    
                    if d < self.reidentification_distance and d < min_dist:
                        min_dist = d
                        best_match = pid
                        time_factor = max(0, 1.0 - (track.frames_missing / self.memory_frames))
                        dist_factor = max(0, 1.0 - (d / self.reidentification_distance))
                        confidence = (time_factor * 0.4 + dist_factor * 0.6)
                
                if best_match and confidence > 0.15:
                    track = self.missing_pieces[best_match]
                    track.update((cx, cy), bbox, frame_num)
                    self.active_pieces[best_match] = track
                    del self.missing_pieces[best_match]
                    matched_tracks.add(best_match)
                    tracked_results.append((color, best_match, cx, cy, bbox, dist, 'spatial', confidence))
                    print(f"Re-identified {best_match} by spatial matching ({confidence:.1%})")
                else:
                    new_id = f"{color} {self.next_id[color]}"
                    self.next_id[color] += 1
                    track = self.TrackData(new_id, color, (cx, cy), bbox, frame_num)
                    self.active_pieces[new_id] = track
                    self.all_pieces_ever_seen[new_id] = track
                    tracked_results.append((color, new_id, cx, cy, bbox, dist, 'new', 1.0))
                    
            else:
                new_id = f"{color} {self.next_id[color]}"
                self.next_id[color] += 1
                track = self.TrackData(new_id, color, (cx, cy), bbox, frame_num)
                self.active_pieces[new_id] = track
                self.all_pieces_ever_seen[new_id] = track
                tracked_results.append((color, new_id, cx, cy, bbox, dist, 'new', 1.0))
        
        return tracked_results
    
    def get_statistics(self):
        black_active = len([p for p in self.active_pieces.values() if 'black' in p.color])
        white_active = len([p for p in self.active_pieces.values() if 'white' in p.color])
        black_missing = len([p for p in self.missing_pieces.values() if 'black' in p.color])
        white_missing = len([p for p in self.missing_pieces.values() if 'white' in p.color])
        
        return {
            'active': len(self.active_pieces),
            'missing': len(self.missing_pieces),
            'total_ever_seen': len(self.all_pieces_ever_seen),
            'black_active': black_active,
            'white_active': white_active,
            'black_missing': black_missing,
            'white_missing': white_missing,
            'camera_moving': self.camera_moving,
        }
    
    def get_piece_manifest(self):
        manifest = {
            'black piece': {'active': [], 'missing': []},
            'white piece': {'active': [], 'missing': []}
        }
        
        for pid, track in self.active_pieces.items():
            manifest[track.color]['active'].append(pid)
        
        for pid, track in self.missing_pieces.items():
            manifest[track.color]['missing'].append(pid)
        
        return manifest


class GridCoordinateSystem:
    """Configurable grid coordinate system for robotic arm navigation"""
    # still in testing phase for the grid system
    def __init__(self, board_width_mm=300, rows=3, cols=3):
        self.board_width_mm = board_width_mm
        self.rows = rows
        self.cols = cols
        self.cell_width_mm = board_width_mm / cols
        self.cell_height_mm = board_width_mm / rows
        # Backward-compatible value for square 3x3 grids / old print statements.
        self.cell_size_mm = self.cell_width_mm
        self.board_corners = None
        self.grid_cells = {}
        self.homography_matrix = None
        self.pixels_per_mm = None
        
    def calibrate(self, board_bbox):
        """Set up the coordinate system based on detected board"""
        x1, y1, x2, y2 = board_bbox
        board_width_px = x2 - x1
        board_height_px = y2 - y1
        
        self.pixels_per_mm = board_width_px / self.board_width_mm
        
        # Define board corners (clockwise from top-left)
        self.board_corners = np.array([
            [x1, y1],  # top-left
            [x2, y1],  # top-right
            [x2, y2],  # bottom-right
            [x1, y2]   # bottom-left
        ], dtype=np.float32)
        
        # Create grid cells (row, col) where (0,0) is top-left
        self.grid_cells = {}
        cell_width = board_width_px / self.cols
        cell_height = board_height_px / self.rows
        
        for row in range(self.rows):
            for col in range(self.cols):
                cell_x1 = x1 + col * cell_width
                cell_y1 = y1 + row * cell_height
                cell_x2 = cell_x1 + cell_width
                cell_y2 = cell_y1 + cell_height
                
                center_x = int((cell_x1 + cell_x2) / 2)
                center_y = int((cell_y1 + cell_y2) / 2)
                
                self.grid_cells[(row, col)] = {
                    'bbox': (int(cell_x1), int(cell_y1), int(cell_x2), int(cell_y2)),
                    'center_px': (center_x, center_y),
                    'center_mm': (col * self.cell_width_mm, row * self.cell_height_mm),
                    'corners': np.array([
                        [cell_x1, cell_y1],
                        [cell_x2, cell_y1],
                        [cell_x2, cell_y2],
                        [cell_x1, cell_y2]
                    ], dtype=np.float32)
                }
        
        return True
    
    def get_grid_position(self, cx, cy):
        """Determine which grid cell contains point (cx, cy)"""
        for coord, cell in self.grid_cells.items():
            x1, y1, x2, y2 = cell['bbox']
            if x1 <= cx <= x2 and y1 <= cy <= y2:
                return coord, cell
        return None, None
    
    def calculate_movement_to_target(self, piece_position_px, target_coord):
        """
        Calculate how far the arm needs to move in X and Y to reach target
        Returns: (forward_mm, right_mm, distance_mm)
        - forward_mm: positive = move forward (down in image)
        - right_mm: positive = move right
        """
        if target_coord not in self.grid_cells or self.pixels_per_mm is None:
            return None, None, None
        
        target_cell = self.grid_cells[target_coord]
        target_px = target_cell['center_px']
        
        # Calculate pixel differences
        dx_px = target_px[0] - piece_position_px[0]  # right is positive
        dy_px = target_px[1] - piece_position_px[1]  # down is positive (forward)
        
        # Convert to millimeters
        right_mm = dx_px / self.pixels_per_mm
        forward_mm = dy_px / self.pixels_per_mm
        
        # Calculate straight-line distance
        distance_mm = np.sqrt(right_mm**2 + forward_mm**2)
        
        return forward_mm, right_mm, distance_mm
    
    def draw_grid(self, frame):
        """Draw the grid overlay on frame"""
        if not self.grid_cells:
            return
        
        # Draw grid lines
        for coord, cell in self.grid_cells.items():
            x1, y1, x2, y2 = cell['bbox']
            
            # Draw cell borders with bright green
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
            
            # Draw center point
            center = cell['center_px']
            cv2.circle(frame, center, 7, (0, 255, 255), -1)
            cv2.circle(frame, center, 10, (0, 255, 0), 3)
            
            # Label with coordinates - larger and more visible
            label = f"({coord[0]},{coord[1]})"
            
            # Draw label background for better visibility
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
            label_x = center[0] - label_size[0]//2
            label_y = center[1] + 30
            
            cv2.rectangle(frame,
                         (label_x - 8, label_y - label_size[1] - 8),
                         (label_x + label_size[0] + 8, label_y + 8),
                         (0, 0, 0), -1)
            
            cv2.rectangle(frame,
                         (label_x - 8, label_y - label_size[1] - 8),
                         (label_x + label_size[0] + 8, label_y + 8),
                         (0, 255, 0), 2)
            
            # Draw label text
            cv2.putText(frame, label,
                       (label_x, label_y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)


class RoboticArmVisionSystem:
    def __init__(self, model_path, board_width_mm=300, piece_diameter_mm=30, 
                 focal_length=3.6, sensor_width_mm=4.8, image_width=1280,
                 home_position=(0, 0), rows=3, cols=3,
                 confidence_threshold=0.6, vote_window=7, min_votes=4):
        """
        Vision system with coordinate tracking for robotic arm
        Args:
            home_position: tuple (row, col) for arm home position, default (0,0)
        """
        self.model = YOLO(model_path)
        self.board_width_mm = board_width_mm
        self.piece_diameter_mm = piece_diameter_mm
        self.focal_length = focal_length
        self.sensor_width_mm = sensor_width_mm
        self.image_width = image_width
        self.home_position = home_position
        self.rows = rows
        self.cols = cols
        self.confidence_threshold = confidence_threshold
        self.vote_window = vote_window
        self.min_votes = min_votes
        
        self.tracker = RoboticArmPieceTracker()
        self.grid = GridCoordinateSystem(board_width_mm, rows=rows, cols=cols)
        self.state_history = deque(maxlen=vote_window)
        self.stable_state = {}
        self.state_is_valid = False
        
        # Calculate focal length in pixels
        self.focal_length_px = (self.focal_length * self.image_width) / self.sensor_width_mm
        
        # Colors
        self.overlay_colors = {
            'board': (100, 255, 100),
            'black piece': (80, 80, 80),
            'white piece': (255, 255, 200)
        }
        
        self.outline_colors = {
            'board': (0, 255, 0),
            'black piece': (0, 0, 0),
            'white piece': (255, 255, 255)
        }
        
        self.frame_count = 0
        self.board_calibrated = False
        
    def estimate_distance(self, object_width_px, is_board=False):
        """Estimate distance using pinhole camera model"""
        if is_board:
            real_width = self.board_width_mm
        else:
            real_width = self.piece_diameter_mm
        
        if object_width_px > 0:
            distance = (real_width * self.focal_length_px) / object_width_px
            return distance
        return None
    
    def process_frame(self, frame):
        """Process frame with grid coordinates and movement calculations"""
        h, w = frame.shape[:2]
        self.frame_count += 1
        
        # Run inference with verbose=False to suppress printing
        results = self.model(frame, verbose=False)[0]
        
        # Create annotated frame
        annotated_frame = frame.copy()
        overlay = frame.copy()
        
        # Draw segmentation masks
        has_masks = results.masks is not None
        if has_masks:
            masks = results.masks.data.cpu().numpy()
            boxes = results.boxes.xyxy.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy()
            confidences = results.boxes.conf.cpu().numpy()
            
            for i, (box, cls, mask, conf) in enumerate(zip(boxes, classes, masks, confidences)):
                if conf < self.confidence_threshold:
                    continue
                class_name = results.names[int(cls)]
                
                # Resize mask
                mask_resized = cv2.resize(mask, (w, h))
                mask_bool = mask_resized > 0.5
                
                # Colors
                fill_color = self.overlay_colors.get(class_name, (0, 255, 255))
                outline_color = self.outline_colors.get(class_name, (0, 255, 255))
                
                # Fill and contours
                overlay[mask_bool] = fill_color
                contours, _ = cv2.findContours(mask_bool.astype(np.uint8), 
                                              cv2.RETR_EXTERNAL, 
                                              cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(annotated_frame, contours, -1, outline_color, 2)
            
            cv2.addWeighted(overlay, 0.3, annotated_frame, 0.7, 0, annotated_frame)
        
        # Detect board and calibrate grid
        board_detected = False
        board_bbox = None
        for box, cls, conf in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf):
            if float(conf) < self.confidence_threshold:
                continue

            x1, y1, x2, y2 = map(int, box)
            class_name = results.names[int(cls)]
            
            if 'board' in class_name.lower():
                board_detected = True
                board_bbox = (x1, y1, x2, y2)
                
                # ALWAYS update grid when board is detected (not just first time)
                self.grid.calibrate(board_bbox)
                
                if not self.board_calibrated:
                    self.board_calibrated = True
                    print(f"Grid calibrated: {self.rows}x{self.cols} grid on {self.board_width_mm}mm board")
                    print(f"Cell size: {self.grid.cell_width_mm:.1f}mm x {self.grid.cell_height_mm:.1f}mm")
                    print(f"Home position: {self.home_position}")
                
                # Draw board label
                board_width_px = x2 - x1
                board_distance = self.estimate_distance(board_width_px, is_board=True)
                label = f"BOARD: {board_distance:.0f}mm" if board_distance else "BOARD"
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                cv2.rectangle(annotated_frame, 
                             (x1, y1 - label_size[1] - 15), 
                             (x1 + label_size[0] + 10, y1), 
                             (0, 255, 0), -1)
                cv2.putText(annotated_frame, label, (x1+5, y1-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                break
        
        # Collect piece detections
        detections = []
        for box, cls, conf in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf):
            if float(conf) < self.confidence_threshold:
                continue

            x1, y1, x2, y2 = map(int, box)
            class_name = results.names[int(cls)]
            
            if 'board' not in class_name.lower():
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                width_px = x2 - x1
                
                color = class_name
                distance_mm = self.estimate_distance(width_px, is_board=False)
                
                detections.append((color, cx, cy, (x1, y1, x2, y2), distance_mm))
        
        # Update tracking
        tracked = self.tracker.update(detections, self.frame_count)
        
        # Visualize pieces with movement instructions
        piece_info = []
        for color, piece_id, cx, cy, (x1, y1, x2, y2), dist, status, confidence in tracked:
            # Get grid position
            grid_coord, grid_cell = self.grid.get_grid_position(cx, cy)
            
            # Calculate movement to home position
            forward_mm, right_mm, total_dist = None, None, None
            if grid_coord and self.board_calibrated:
                forward_mm, right_mm, total_dist = self.grid.calculate_movement_to_target(
                    (cx, cy), self.home_position)
                
                # Debug: print movement calculation occasionally
                if self.frame_count % 60 == 0:
                    print(f"  {piece_id} at grid {grid_coord}: pixel ({cx},{cy}) -> " +
                          f"Forward {forward_mm:+.0f}mm, Right {right_mm:+.0f}mm")
            
            # Determine box color
            if status == 'eliminated':
                box_color = (0, 255, 255)
                thickness = 4
            elif status == 'spatial':
                box_color = (255, 165, 0)
                thickness = 3
            elif status == 'new':
                box_color = (255, 0, 255)
                thickness = 3
            else:
                box_color = (0, 255, 0)
                thickness = 2
            
            # Draw bounding box and center
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), box_color, thickness)
            cv2.circle(annotated_frame, (cx, cy), 6, (0, 255, 255), -1)
            cv2.circle(annotated_frame, (cx, cy), 8, box_color, 2)
            
            # Draw line to home position if calibrated and not at home
            if self.board_calibrated and self.home_position in self.grid.grid_cells:
                if grid_coord != self.home_position:  # Only draw if not at home
                    home_center = self.grid.grid_cells[self.home_position]['center_px']
                    cv2.arrowedLine(annotated_frame, (cx, cy), home_center, 
                                  (255, 0, 255), 2, tipLength=0.15)
            
            # Create comprehensive label
            labels = [piece_id]
            if grid_coord:
                labels.append(f"Grid: ({grid_coord[0]},{grid_coord[1]})")
            
            if forward_mm is not None and right_mm is not None:
                # Movement instructions
                fwd_dir = "FWD" if forward_mm >= 0 else "BACK"
                right_dir = "RIGHT" if right_mm >= 0 else "LEFT"
                labels.append(f"{fwd_dir}: {abs(forward_mm):.0f}mm")
                labels.append(f"{right_dir}: {abs(right_mm):.0f}mm")
                labels.append(f"Total: {total_dist:.0f}mm")
            
            # Draw multi-line label
            y_offset = y1 - 10
            for label in reversed(labels):
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                
                # Background
                cv2.rectangle(annotated_frame,
                             (x1, y_offset - label_size[1] - 5),
                             (x1 + label_size[0] + 10, y_offset),
                             box_color, -1)
                
                # Text
                text_color = (0, 0, 0) if box_color in [(255, 255, 255), (0, 255, 255)] else (255, 255, 255)
                cv2.putText(annotated_frame, label, (x1+5, y_offset-5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
                
                y_offset -= label_size[1] + 8
            
            piece_info.append({
                'id': piece_id,
                'color': color,
                'position': (cx, cy),
                'grid_coord': grid_coord,
                'bbox': (x1, y1, x2, y2),
                'distance_mm': dist,
                'movement_to_home': {
                    'forward_mm': forward_mm,
                    'right_mm': right_mm,
                    'total_mm': total_dist
                },
                'status': status,
                'confidence': confidence
            })
        
        # Build a per-frame symbolic board state and stabilize it with temporal voting.
        frame_state = {}
        for piece in piece_info:
            if piece["grid_coord"] is not None:
                frame_state[piece["grid_coord"]] = piece["color"]

        self.state_history.append(frame_state)
        self.stable_state = self.get_stable_state()
        self.state_is_valid = self.validate_state(self.stable_state)

        if len(self.state_history) >= self.min_votes and not self.state_is_valid:
            print("Invalid or unstable state, waiting for better frames...")

        # Draw grid AFTER pieces so it's visible on top
        if self.board_calibrated and board_bbox:
            self.grid.draw_grid(annotated_frame)
        
        # Draw info overlay
        self._draw_overlay(annotated_frame, piece_info, board_detected)
        
        return piece_info, annotated_frame
    
    def get_stable_state(self):
        """Return a majority-voted board state from the recent frame history."""
        votes = {}

        for state in self.state_history:
            for cell, color in state.items():
                votes.setdefault(cell, []).append(color)

        stable = {}
        for cell, colors in votes.items():
            color, count = Counter(colors).most_common(1)[0]
            if count >= self.min_votes:
                stable[cell] = color

        return stable

    def validate_state(self, state):
        """Reject impossible or obviously corrupted board states before robot action."""
        valid_colors = {"black piece", "white piece"}

        for (row, col), color in state.items():
            if not (0 <= row < self.rows and 0 <= col < self.cols):
                return False
            if color not in valid_colors:
                return False

        black = sum(color == "black piece" for color in state.values())
        white = sum(color == "white piece" for color in state.values())

        # For alternating-turn games, piece counts should stay close.
        if abs(black - white) > 1:
            return False

        # Also prevent impossible overcrowding.
        if len(state) > self.rows * self.cols:
            return False

        return True

    def _draw_overlay(self, frame, pieces, board_detected):
        """Draw information overlay"""
        h, w = frame.shape[:2]
        stats = self.tracker.get_statistics()
        
        # Calculate overlay height
        overlay_height = min(250 + len(pieces) * 20, h - 20)
        
        # Semi-transparent background
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (500, overlay_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Title
        y = 35
        cv2.putText(frame, "Robotic Arm Vision System", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        y += 35
        
        # Grid status
        if self.board_calibrated:
            cv2.putText(frame, f"Grid: {self.rows}x{self.cols} calibrated | Home: {self.home_position}", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        else:
            cv2.putText(frame, "Waiting for board detection", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        y += 28

        state_status = "valid" if self.state_is_valid else "unstable/invalid"
        cv2.putText(frame, f"Stable State: {len(self.stable_state)} cells | {state_status}", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if self.state_is_valid else (0, 165, 255), 1)
        y += 28
        
        # Camera status
        if stats['camera_moving']:
            cv2.putText(frame, "CAMERA MOVING", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
            y += 28
        
        # Piece inventory
        cv2.putText(frame, f"Black: {stats['black_active']} {stats['black_missing']} | " +
                          f"White: {stats['white_active']} {stats['white_missing']}",
                   (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y += 30
        
        # Legend
        cv2.putText(frame, "Movement Legend:", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y += 20
        
        cv2.putText(frame, "FWD/BACK = Forward/Backward (Y-axis)", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y += 18
        
        cv2.putText(frame, "RIGHT/LEFT = Right/Left (X-axis)", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y += 18
        
        cv2.putText(frame, f"Purple arrow = Path to home {self.home_position}", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y += 25
        
        # Piece list with movement data
        if pieces and y < overlay_height - 40:
            cv2.putText(frame, "Tracked Pieces:", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y += 20
            
            for piece in sorted(pieces, key=lambda p: p['id']):
                if y >= overlay_height - 20:
                    break
                    
                coord = piece['grid_coord']
                move = piece['movement_to_home']
                
                if coord and move['total_mm']:
                    piece_text = f"{piece['id']} at ({coord[0]},{coord[1]}): " + \
                                f"→ {abs(move['forward_mm']):.0f}mm " + \
                                f"{'↑' if move['forward_mm'] < 0 else '↓'} " + \
                                f"{abs(move['right_mm']):.0f}mm " + \
                                f"{'←' if move['right_mm'] < 0 else '→'}"
                else:
                    piece_text = f"{piece['id']}: detecting position"
                
                cv2.putText(frame, piece_text, (30, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)
                y += 18


def main():
    """Main function"""
    
    # Model path (please use the path that you are using!!!!)
    model_path = r'C:\Users\gamescrafters\cv\chess.v2-new-images.yolov9\runs\segment\train4\weights\best.pt'
    
    # Initialize vision system
    vision = RoboticArmVisionSystem(
        model_path=model_path,
        board_width_mm=300,
        piece_diameter_mm=30,
        focal_length=3.6,
        sensor_width_mm=4.8,
        image_width=1280,
        home_position=(0, 0),  # Top-left corner
        rows=3,
        cols=3,
        confidence_threshold=0.6,
        vote_window=7,
        min_votes=4
    )
    
    # Start webcam
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    if not cap.isOpened():
        print("ERROR: Could not open webcam!")
        return
    
    print("\nWebcam started! Show the board to calibrate grid")
    print("The 3x3 grid will appear in BRIGHT GREEN once calibrated.")
    print("Each cell will show coordinates like (0,0), (0,1), etc.\n")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Process frame
        pieces, annotated = vision.process_frame(frame)
        
        # Display
        cv2.imshow('Robotic Arm Vision', annotated)
        
        # Handle keyboard
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f"grid_tracking_{int(time.time())}.jpg"
            cv2.imwrite(filename, annotated)
            print(f"\nScreenshot saved: {filename}")
        elif key == ord('r'):
            vision.tracker = RoboticArmPieceTracker()
            vision.board_calibrated = False
            vision.state_history.clear()
            vision.stable_state = {}
            vision.state_is_valid = False
            print("\nTracking, temporal state, and calibration reset")
        elif key == ord('i'):
            # Print detailed position info
            
            for piece in sorted(pieces, key=lambda p: p['id']):
                print(f"\n{piece['id']}:")
                if piece['grid_coord']:
                    print(f"  Current Grid: ({piece['grid_coord'][0]}, {piece['grid_coord'][1]})")
                    move = piece['movement_to_home']
                    if move['forward_mm'] is not None:
                        print(f"  To reach home {vision.home_position}:")
                        print(f"    Forward/Back: {move['forward_mm']:+.1f}mm ({'forward' if move['forward_mm'] > 0 else 'backward'})")
                        print(f"    Right/Left: {move['right_mm']:+.1f}mm ({'right' if move['right_mm'] > 0 else 'left'})")
                        print(f"    Total Distance: {move['total_mm']:.1f}mm")
                else:
                    print("  Position: Outside grid")
           
        elif key == ord('h'):
            # Set new home position
            print("\nEnter new home position:")
            try:
                row = int(input("  Row (0-2): "))
                col = int(input("  Col (0-2): "))
                if 0 <= row < vision.rows and 0 <= col < vision.cols:
                    vision.home_position = (row, col)
                    print(f"Home position set to ({row}, {col})")
                else:
                    print(f"Invalid coordinates! Row must be 0-{vision.rows - 1}, col must be 0-{vision.cols - 1}")
            except:
                print("Invalid input!")
    
    cap.release()
    cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main()