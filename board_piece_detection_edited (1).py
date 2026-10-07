
import cv2
import numpy as np
from ultralytics import YOLO

class SegmentationDetector:
    def __init__(self, model_path, confidence_threshold=0.6):
        """
        Initialize the segmentation detector
        
        Args:
            model_path: Path to your trained YOLO segmentation model
        """
        print("Loading YOLO segmentation model")
        self.model = YOLO(model_path)
        print("Model loaded")
        self.confidence_threshold = confidence_threshold
        
        # Define colors for each class
        self.colors = {
            'board': (0, 255, 0),      # Green
            'black piece': (0, 0, 0),   # Black
            'white piece': (255, 255, 255)  # White
        }
        
        # Define overlay colors with transparency
        self.overlay_colors = {
            'board': (100, 255, 100),      # Light green
            'black piece': (80, 80, 80),   # Dark gray
            'white piece': (255, 255, 200)  # Light yellow-white
        }
    
    def process_frame(self, frame):
        """
        Process a single frame and return annotated version
        
        Args:
            frame: Input frame from webcam
            
        Returns:
            annotated_frame: Frame with segmentation overlay and labels
            detection_info: Dictionary with detection details
        """
        h, w = frame.shape[:2]
        
        # Run inference
        results = self.model(frame)[0]
        
        # Create a copy for annotation
        annotated_frame = frame.copy()
        overlay = frame.copy()
        
        detection_info = {
            'board': 0,
            'black_pieces': 0,
            'white_pieces': 0,
            'total_detections': 0
        }
        
        # Check if masks exist
        if results.masks is None:
            cv2.putText(annotated_frame, "No detections", (20, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return annotated_frame, detection_info
        
        # Get masks and boxes
        masks = results.masks.data.cpu().numpy()
        boxes = results.boxes.xyxy.cpu().numpy()
        classes = results.boxes.cls.cpu().numpy()
        confidences = results.boxes.conf.cpu().numpy()
        
        # Process each detection
        for i, (box, cls, mask, conf) in enumerate(zip(boxes, classes, masks, confidences)):
            if conf < self.confidence_threshold:
                continue

            class_name = results.names[int(cls)]
            x1, y1, x2, y2 = map(int, box)
            
            # Resize mask to frame size
            mask_resized = cv2.resize(mask, (w, h))
            mask_bool = mask_resized > 0.5
            
            # Get colors for this class
            outline_color = self.colors.get(class_name, (0, 255, 255))
            fill_color = self.overlay_colors.get(class_name, (0, 255, 255))
            
            # Fill the segmented area with color
            overlay[mask_bool] = fill_color
            
            # Draw contours around the mask
            contours, _ = cv2.findContours(mask_bool.astype(np.uint8), 
                                          cv2.RETR_EXTERNAL, 
                                          cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(annotated_frame, contours, -1, outline_color, 3)
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), outline_color, 2)
            
            # Add label with confidence
            label = f"{class_name}: {conf:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            
            # Draw label background
            cv2.rectangle(annotated_frame, 
                         (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), 
                         outline_color, -1)
            
            # Draw label text
            text_color = (255, 255, 255) if class_name != 'white piece' else (0, 0, 0)
            cv2.putText(annotated_frame, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
            
            # Update detection counts
            if 'board' in class_name.lower():
                detection_info['board'] += 1
            elif 'black' in class_name.lower():
                detection_info['black_pieces'] += 1
            elif 'white' in class_name.lower():
                detection_info['white_pieces'] += 1
            
            detection_info['total_detections'] += 1

        if detection_info['total_detections'] == 0:
            cv2.putText(annotated_frame, f"No detections above {self.confidence_threshold:.2f}", (20, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        
        # Blend overlay with original frame
        cv2.addWeighted(overlay, 0.4, annotated_frame, 0.6, 0, annotated_frame)
        
        # Add detection summary
        self._draw_summary(annotated_frame, detection_info)
        
        return annotated_frame, detection_info
    
    def _draw_summary(self, frame, info):
        """Draw detection summary on frame"""
        h, w = frame.shape[:2]
        
        # Create semi-transparent background
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (300, 130), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        
        # Draw text
        y_pos = 35
        cv2.putText(frame, "Detection Summary", (20, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        y_pos += 30
        
        cv2.putText(frame, f"Board: {info['board']}", (20, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
        y_pos += 25
        
        cv2.putText(frame, f"Black Pieces: {info['black_pieces']}", (20, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
        y_pos += 25
        
        cv2.putText(frame, f"White Pieces: {info['white_pieces']}", (20, y_pos),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 200), 1)


def main():
    """Main function"""
    
    # Path to trained model (please change it if it changed to the path you are using!)
    model_path = r'C:\Users\gamescrafters\cv_robotic_arm\chess.v2-new-images.yolov9\runs\segment\train4\weights\best.pt'
    
    # Initialize detector
    detector = SegmentationDetector(model_path)
    
    # Open webcam (change this to whatever camera is being used such as the elephant camera)
    cap = cv2.VideoCapture(0)
    
    # Set camera resolution  (set the resolution based on camera specs)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Process frame
        annotated, info = detector.process_frame(frame)
        
        # Print info every 60 frames 
        if frame_count % 60 == 0:
            print(f"\nFrame {frame_count}:")
            print(f"  Board detected: {info['board']}")
            print(f"  Black pieces: {info['black_pieces']}")
            print(f"  White pieces: {info['white_pieces']}")
            print(f"  Total objects: {info['total_detections']}")
        
        # Display
        cv2.imshow('YOLO Segmentation Detector', annotated)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("\nExiting...")
            break
        elif key == ord('s'):
            filename = f"detection_{frame_count}.jpg"
            cv2.imwrite(filename, annotated)
            print(f"\nScreenshot saved: {filename}")
        
        frame_count += 1
    
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()