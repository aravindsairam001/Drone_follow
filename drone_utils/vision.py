"""
Vision utilities for drone object detection and tracking
"""

import cv2
import numpy as np
from ultralytics import YOLO
import sys
import time

from .config import TARGET_CLASS, FONT, GREEN, RED, ARROW_COLOR
from .config import CENTER_TOLERANCE, MICRO_ZONE, AREA_TOLERANCE, AREA_MICRO_ZONE
from .config import P_GAIN_POSITION, P_GAIN_AREA, MAX_CORRECTION_RATE
from .config import TESTING_MODE, SAFETY_SCALE
from . import state_manager

# Load YOLOv8 model (use yolov8n for speed)
def load_model(model_path="yolov8n.pt"):
    """Load the YOLO model for object detection"""
    try:
        model = YOLO(model_path)
        print("YOLO model loaded successfully")
        return model
    except Exception as e:
        print(f"Error loading YOLO model: {e}")
        sys.exit(1)

def setup_camera(esp32_stream_url='http://192.168.43.43/stream'):
    """Set up camera connection, trying ESP32 first, then webcam"""
    print(f"Connecting to ESP32 camera at {esp32_stream_url}...")
    
    # First try ESP32 camera
    try:
        cap = cv2.VideoCapture(esp32_stream_url)
        if not cap.isOpened():
            print("Could not connect to ESP32 camera, falling back to webcam")
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                print("ERROR: Could not open any camera!")
                sys.exit(1)
            
            # Verify we can get at least one frame from webcam
            ret, _ = cap.read()
            if not ret:
                print("WARNING: Could not read frame from webcam, trying again...")
                cap.release()
                time.sleep(0.5)
                cap = cv2.VideoCapture(0)
                if not cap.isOpened():
                    print("ERROR: Could not reopen webcam!")
                    sys.exit(1)
        
        # Set camera properties
        cap.set(cv2.CAP_PROP_FPS, 30)
        print("Camera connected successfully")
        return cap
    except Exception as e:
        print(f"Error connecting to camera: {e}")
        sys.exit(1)

def detect_objects(frame, model):
    """Detect objects in a frame using the YOLO model"""
    results = model.predict(frame, conf=0.5, iou=0.5, classes=[TARGET_CLASS], verbose=False)
    detections = results[0].boxes.data.cpu().numpy()
    return detections

def initialize_target(detections):
    """Initialize the tracking target from detections"""
    if len(detections) > 0:
        # Pick the largest person detected
        target_box = max(detections, key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))
        x1, y1, x2, y2, conf, cls = target_box[:6]
        state_manager.target_id = 1  # Simulated ID
        state_manager.initial_box = (int(x1), int(y1), int(x2), int(y2))
        state_manager.initial_area = (x2 - x1) * (y2 - y1)
        state_manager.INITIALIZED = True
        print("[INFO] Human locked.")
        return True
    return False

def track_target(detections):
    """Track the initialized target in new detections"""
    best_match = None
    min_dist = float('inf')
    
    init_cx = (state_manager.initial_box[0] + state_manager.initial_box[2]) // 2
    init_cy = (state_manager.initial_box[1] + state_manager.initial_box[3]) // 2
    
    for det in detections:
        x1, y1, x2, y2, conf, cls = det[:6]
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        area = (x2 - x1) * (y2 - y1)

        dist = np.sqrt((init_cx - cx)**2 + (init_cy - cy)**2)

        if dist < min_dist:
            min_dist = dist
            best_match = (int(x1), int(y1), int(x2), int(y2), area, cx, cy)
            
    return best_match

def draw_tracking_visualization(frame, target, frame_center):
    """Draw tracking visualization elements on the frame"""
    x1, y1, x2, y2, area, cx, cy = target
    
    # Draw bounding box and target dot
    cv2.rectangle(frame, (x1, y1), (x2, y2), GREEN, 2)
    dot_size = int(15 * np.sqrt(area / state_manager.initial_area))
    cv2.circle(frame, (cx, cy), dot_size, GREEN, -1)

    # Drone camera center
    cv2.circle(frame, frame_center, 10, RED, -1)
    
    # Draw vector line with arrow from center to target
    cv2.arrowedLine(frame, frame_center, (cx, cy), ARROW_COLOR, 2, tipLength=0.2)
    
    # Draw outer tolerance zone (standard correction zone)
    cv2.rectangle(frame, 
                (frame_center[0] - CENTER_TOLERANCE, frame_center[1] - CENTER_TOLERANCE),
                (frame_center[0] + CENTER_TOLERANCE, frame_center[1] + CENTER_TOLERANCE),
                (0, 165, 255), 1)
    
    # Draw inner micro-adjustment zone (precision targeting)
    cv2.rectangle(frame, 
                (frame_center[0] - MICRO_ZONE, frame_center[1] - MICRO_ZONE),
                (frame_center[0] + MICRO_ZONE, frame_center[1] + MICRO_ZONE),
                (0, 255, 255), 1)
    
    # Target size indicators
    target_area_text = f"Area: {area:.0f} / {state_manager.initial_area:.0f}"
    cv2.putText(frame, target_area_text, (cx - 50, cy + 30), FONT, 0.5, GREEN, 1)
    
    return frame

def analyze_tracking(target, frame_center, frame_size):
    """Analyze tracking data and generate movement commands"""
    w, h = frame_size
    x1, y1, x2, y2, area, cx, cy = target
    
    # Calculate offsets
    dx = cx - frame_center[0]  # Horizontal offset from center
    dy = cy - frame_center[1]  # Vertical offset from center
    da = area - state_manager.initial_area   # Area difference (proxy for distance)
    
    # Calculate tracking error for visualization and logging
    tracking_error = np.sqrt(dx**2 + dy**2)
    area_error_pct = abs(da) / max(state_manager.initial_area, 100) * 100
    
    # Multi-zone precision targeting with combined command generation
    command = []
    intensities = []
    
    # Horizontal positioning (X-axis)
    if abs(dx) > CENTER_TOLERANCE:
        # Outside main tolerance zone - stronger correction
        move = "LEFT" if dx < 0 else "RIGHT"
        command.append(move)
        # Proportional control based on distance from center
        base_intensity = min(abs(dx) / (w/3), 1.0) * P_GAIN_POSITION
        # Apply non-linear response curve for more precise control
        intensity = min(base_intensity ** 1.3, MAX_CORRECTION_RATE)
        intensities.append(intensity)
    elif abs(dx) > MICRO_ZONE:
        # Fine positioning zone - gentler corrections
        move = "LEFT" if dx < 0 else "RIGHT"
        command.append(move)
        # Micro-adjustments with very low intensity
        intensity = min(abs(dx) / (w/2), 0.3) * P_GAIN_POSITION * 0.6
        intensities.append(intensity)
        
    # Vertical positioning (Y-axis)
    if abs(dy) > CENTER_TOLERANCE:
        # Outside main tolerance zone - stronger correction
        move = "UP" if dy < 0 else "DOWN"
        command.append(move)
        # Proportional control based on distance from center
        base_intensity = min(abs(dy) / (h/3), 1.0) * P_GAIN_POSITION
        # Apply non-linear response curve for more precise control
        intensity = min(base_intensity ** 1.3, MAX_CORRECTION_RATE)
        intensities.append(intensity)
    elif abs(dy) > MICRO_ZONE:
        # Fine positioning zone - gentler corrections
        move = "UP" if dy < 0 else "DOWN"
        command.append(move)
        # Micro-adjustments with very low intensity
        intensity = min(abs(dy) / (h/2), 0.3) * P_GAIN_POSITION * 0.6
        intensities.append(intensity)
        
    # Depth positioning (Z-axis / distance)
    if abs(da) > AREA_TOLERANCE:
        # Outside main tolerance zone - stronger correction
        move = "FORWARD" if da < 0 else "BACKWARD"
        command.append(move)
        # Proportional control based on area difference
        area_ratio = abs(da) / max(state_manager.initial_area, 100)
        # More aggressive scaling for distance adjustments
        intensity = min(area_ratio * 2.0 * P_GAIN_AREA, MAX_CORRECTION_RATE)
        intensities.append(intensity)
    elif abs(da) > AREA_MICRO_ZONE:
        # Fine depth positioning zone
        move = "FORWARD" if da < 0 else "BACKWARD"
        command.append(move)
        # Micro-adjustments for distance
        area_ratio = abs(da) / max(state_manager.initial_area, 100)
        intensity = min(area_ratio * 1.0 * P_GAIN_AREA * 0.5, 0.3)
        intensities.append(intensity)
        
    # If no corrections needed, hover in place
    if not command:
        command = ["HOVER"]
        intensities = [0.0]
    
    tracking_metrics = {
        'dx': dx,
        'dy': dy,
        'da': da,
        'tracking_error': tracking_error,
        'area_error_pct': area_error_pct
    }
    
    return command, intensities, tracking_metrics

def draw_movement_indicators(frame, command, intensities, tracking_metrics, frame_center):
    """Draw movement visualization elements on the frame"""
    w, h = frame.shape[1], frame.shape[0]
    
    # Movement visualization
    if command[0] != "HOVER":
        # Enhanced visual feedback for roll and pitch (left/right, forward/backward)
        vector_length = 30  # Base vector length
        
        # Check for "LEFT" or "RIGHT" commands
        for i, cmd in enumerate(command):
            if cmd in ["LEFT", "RIGHT"]:
                # Calculate vector length based on intensity
                intensity = intensities[i]
                roll_vector_len = int(vector_length * min(intensity * 2, 1.5))
                
                # Draw arrow with length proportional to movement intensity
                if cmd == "LEFT":  # Left movement
                    start_x = w//2 - 5
                    end_x = max(w//2 - roll_vector_len - 15, 5)
                    cv2.arrowedLine(frame, (start_x, h//2), (end_x, h//2), 
                                  ARROW_COLOR, 2, tipLength=0.3)
                    direction = "L"
                else:  # Right movement
                    start_x = w//2 + 5
                    end_x = min(w//2 + roll_vector_len + 15, w-5)
                    cv2.arrowedLine(frame, (start_x, h//2), (end_x, h//2), 
                                  ARROW_COLOR, 2, tipLength=0.3)
                    direction = "R"
                    
                # Show command strength percentage
                strength_pct = int(intensity * 100)
                cv2.putText(frame, f"{direction}:{strength_pct}%", 
                          (w//2 + (30 if cmd == "RIGHT" else -60), h//2 - 10), 
                          FONT, 0.6, ARROW_COLOR, 2)
            
            # Check for "FORWARD" or "BACKWARD" commands
            elif cmd in ["FORWARD", "BACKWARD"]:
                # Calculate vector length based on intensity
                intensity = intensities[i]
                pitch_vector_len = int(vector_length * min(intensity * 2, 1.5))
                
                # Draw arrow with length proportional to movement intensity
                if cmd == "FORWARD":  # Forward movement
                    start_y = h//2 - 5
                    end_y = max(h//2 - pitch_vector_len - 15, 5)
                    cv2.arrowedLine(frame, (w//2, start_y), (w//2, end_y), 
                                  ARROW_COLOR, 2, tipLength=0.3)
                    direction = "F"
                else:  # Backward movement
                    start_y = h//2 + 5
                    end_y = min(h//2 + pitch_vector_len + 15, h-5)
                    cv2.arrowedLine(frame, (w//2, start_y), (w//2, end_y), 
                                  ARROW_COLOR, 2, tipLength=0.3)
                    direction = "B"
                    
                # Show command strength percentage
                strength_pct = int(intensity * 100)
                cv2.putText(frame, f"{direction}:{strength_pct}%", 
                          (w//2 + 10, h//2 + (30 if cmd == "BACKWARD" else -30)), 
                          FONT, 0.6, ARROW_COLOR, 2)
    else:
        # Display "HOVER" indicator when perfectly centered
        cv2.putText(frame, "HOVER", (w//2 - 30, h//2 - 30), FONT, 0.7, (0, 255, 0), 2)
    
    # Status line showing commands
    status_line = f"Command: {' | '.join(command)}"
    if TESTING_MODE:
        status_line = f"TEST ({SAFETY_SCALE*100:.0f}%) | " + status_line
    cv2.putText(frame, status_line, (10, 30), FONT, 0.7, (0, 255, 255), 2)

    # Enhanced tracking metrics with colored status indicators
    dx = tracking_metrics['dx']
    dy = tracking_metrics['dy']
    tracking_error = tracking_metrics['tracking_error']
    area_error_pct = tracking_metrics['area_error_pct']
    
    # Position accuracy indicator (color changes based on tracking quality)
    if tracking_error < MICRO_ZONE:
        error_color = (0, 255, 0)  # Green - excellent tracking
        tracking_status = "PRECISE"
    elif tracking_error < CENTER_TOLERANCE:
        error_color = (0, 255, 255)  # Yellow - good tracking
        tracking_status = "GOOD"
    else:
        error_color = (0, 69, 255)  # Red - needs correction
        tracking_status = "ADJUSTING"
        
    # Detailed position error readouts
    position_error = f"Error: {tracking_error:.1f}px [{tracking_status}]"
    area_error_text = f"Depth: {area_error_pct:.1f}%"
    
    cv2.putText(frame, position_error, (w-240, h-30), FONT, 0.6, error_color, 2)
    cv2.putText(frame, area_error_text, (w-240, h-10), FONT, 0.6, error_color, 2)
    
    # Get target position for dx/dy display
    if len(command) > 0 and command[0] != "HOVER":
        target_pos = (tracking_metrics['dx'] + frame_center[0], tracking_metrics['dy'] + frame_center[1])
        
        # Display offset values near the target
        dx_text = f"dx: {dx:+.1f}"
        dy_text = f"dy: {dy:+.1f}"
        
        cv2.putText(frame, dx_text, (int(target_pos[0]) + 20, int(target_pos[1])), FONT, 0.5, (255, 255, 0), 1)
        cv2.putText(frame, dy_text, (int(target_pos[0]) + 20, int(target_pos[1]) + 15), FONT, 0.5, (255, 255, 0), 1)
    
    # Draw accuracy circle that shows how close we are to perfect centering
    accuracy_radius = max(5, int(tracking_error / 2))
    cv2.circle(frame, frame_center, accuracy_radius, error_color, 1)
    
    return frame
