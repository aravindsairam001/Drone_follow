#!/usr/bin/env python3

import cv2
import numpy as np
from ultralytics import YOLO
import time
import sys
import os

# Import utility modules
from drone_utils import config
from drone_utils import networking
from drone_utils import state_manager
from drone_utils import vision
from drone_utils import commands
from drone_utils import movement
from drone_utils import control_sequences

# Print startup message (just like the original)
print(f"ESP-Drone Human Follower - Starting {time.strftime('%Y-%m-%d %H:%M:%S')}")

# Load YOLOv8 model (use yolov8n for speed)
try:
    model = YOLO("yolov8n.pt")
    print("YOLO model loaded successfully")
except Exception as e:
    print(f"Error loading YOLO model: {e}")
    sys.exit(1)

# ESP32 camera stream URL
esp32_stream_url = config.ESP32_STREAM_URL

# Try to connect to camera
print(f"Connecting to ESP32 camera at {esp32_stream_url}...")
try:
    cap = cv2.VideoCapture(esp32_stream_url)
    if not cap.isOpened():
        print("Could not connect to ESP32 camera, falling back to webcam")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("ERROR: Could not open any camera!")
            sys.exit(1)
    
    # Set camera properties
    cap.set(cv2.CAP_PROP_FPS, 30)
    print("Camera connected successfully")
except Exception as e:
    print(f"Error connecting to camera: {e}")
    sys.exit(1)

# Drone UDP details
print(f"Drone configured at {config.DRONE_IP}:{config.DRONE_PORT}")

# Initialize UDP socket
networking.initialize_udp_socket()

# Main loop
while True:
    # Capture frame from camera
    start_time = time.time()
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame - retrying...")
        time.sleep(0.5)
        continue
    
    # Resize for better performance
    frame_resized = cv2.resize(frame, (640, 480))
    h, w = frame_resized.shape[:2]
    frame_center = (w // 2, h // 2)
    
    # Draw status indicators
    status_text = []
    if state_manager.drone_armed:
        status_text.append("ARMED")
    if state_manager.altitude_hold:
        status_text.append("ALT HOLD")
    if state_manager.follow_mode:
        status_text.append("FOLLOW MODE")
    if config.TESTING_MODE:
        status_text.append(f"TEST {int(config.SAFETY_SCALE*100)}%")
        
    if not status_text:
        status_text = ["DISARMED"]
        status_color = config.RED
    else:
        status_color = config.GREEN
    
    # Draw status indicator dot and text
    cv2.circle(frame_resized, (20, 60), 10, status_color, -1)
    cv2.putText(frame_resized, " | ".join(status_text), (40, 65), config.FONT, 0.6, status_color, 2)
    
    # Process arming sequence visualization if needed
    if state_manager.arm_start_time and not state_manager.ARMING_SEQ_COMPLETE:
        time_since_arm = time.time() - state_manager.arm_start_time
        # Visual indicator of arming sequence
        progress = min(time_since_arm / config.ARM_TIME, 1.0)
        bar_width = int(w * 0.6)
        bar_progress = int(bar_width * progress)
        cv2.rectangle(frame_resized, (w//2 - bar_width//2, h//2 - 30), 
                     (w//2 - bar_width//2 + bar_progress, h//2 - 20), 
                     (0, 255, 0), -1)
        cv2.rectangle(frame_resized, (w//2 - bar_width//2, h//2 - 30), 
                     (w//2 + bar_width//2, h//2 - 20), 
                     (255, 255, 255), 2)
        cv2.putText(frame_resized, "ARMING SEQUENCE", (w//2 - 100, h//2 - 40), 
                   config.FONT, 0.7, (0, 255, 255), 2)
        
        # Complete arming sequence but don't take off yet
        if time_since_arm >= config.ARM_TIME:
            state_manager.ARMING_SEQ_COMPLETE = True
            print("Drone armed and ready! Press 'f' to begin takeoff and following.")
    
    # Skip detection if follow mode is not active
    if not state_manager.follow_mode:
        # Add text to indicate standby mode
        cv2.putText(frame_resized, "STANDBY MODE - Press 'f' to start follow", 
                  (w//2 - 200, h//2), config.FONT, 0.7, (0, 165, 255), 2)
        cv2.imshow("Drone View", frame_resized)
        
        # Check for keyboard input
        key = cv2.waitKey(1) & 0xFF
        
        # Handle keyboard input
        if key == ord('q'):  # Quit
            print("\nExiting program...")
            networking.close_udp_socket()
            cv2.destroyAllWindows()
            sys.exit(0)
            
        elif key == ord('a') and not state_manager.drone_armed:  # Arm
            print("Arming drone...")
            control_sequences.arm_drone()
            
        elif key == ord('f'):  # Follow
            if state_manager.drone_armed:
                if not state_manager.altitude_hold:
                    print("Starting altitude hold mode (takeoff)...")
                    control_sequences.start_altitude_hold()
                
                print("Activating follow mode...")
                state_manager.follow_mode = True
                state_manager.INITIALIZED = False  # Reset target to acquire new one
            else:
                print("Cannot start follow mode: Drone not armed.")
                
        elif key == ord('h'):  # Hover
            print("Emergency hover command")
            movement.send_hover_command()
            
        elif key == ord('l'):  # Land
            print("Landing...")
            control_sequences.land_drone()
            state_manager.drone_armed = False
            state_manager.follow_mode = False
            state_manager.altitude_hold = False
            state_manager.ARMING_SEQ_COMPLETE = False
            
        elif key == ord('d'):  # Diagnostics
            print("Running diagnostics...")
            networking.run_diagnostics()
            
        elif key == ord('r'):  # Reset
            print("Resetting state...")
            state_manager.reset_state()
            
        elif key == ord('t'):  # Toggle test mode
            config.TESTING_MODE = not config.TESTING_MODE
            print(f"Test mode: {'ON' if config.TESTING_MODE else 'OFF'}")
            
        elif key == ord('+') and config.TESTING_MODE:  # Increase safety scale
            config.SAFETY_SCALE = min(config.SAFETY_SCALE + 0.1, 1.0)
            print(f"Safety scaling increased to {config.SAFETY_SCALE*100:.0f}%")
            
        elif key == ord('-') and config.TESTING_MODE:  # Decrease safety scale
            config.SAFETY_SCALE = max(config.SAFETY_SCALE - 0.1, 0.1)
            print(f"Safety scaling decreased to {config.SAFETY_SCALE*100:.0f}%")
            
        continue

    # Perform object detection
    detections = vision.detect_objects(frame_resized, model)

    # If not initialized, try to lock onto a target
    if not state_manager.INITIALIZED:
        vision.initialize_target(detections)
    else:
        # Track the existing target
        target = vision.track_target(detections)
        
        if target is not None:
            # Draw tracking visualization
            frame_resized = vision.draw_tracking_visualization(frame_resized, target, frame_center)
            
            # Analyze tracking data and generate movement commands
            command, intensities, tracking_metrics = vision.analyze_tracking(target, frame_center, (w, h))
            
            # Draw movement indicators
            frame_resized = vision.draw_movement_indicators(frame_resized, command, intensities, 
                                                          tracking_metrics, frame_center)
            
            # Execute the movement command
            movement.execute_movement(command, intensities)
        else:
            cv2.putText(frame_resized, "Target Lost: Hovering", (10, 30), config.FONT, 0.7, (0, 0, 255), 2)
            movement.send_hover_command()

    # Show FPS
    end_time = time.time()
    fps = 1 / (end_time - start_time)
    cv2.putText(frame_resized, f"FPS: {fps:.2f}", (500, 30), config.FONT, 0.6, (255, 255, 255), 1)

    # Display the frame
    cv2.imshow("Drone View", frame_resized)
    
    # Check for keyboard input
    key = cv2.waitKey(1) & 0xFF
    
    # Handle keyboard input
    if key == ord('q'):  # Quit
        print("\nExiting program...")
        networking.close_udp_socket()
        cv2.destroyAllWindows()
        sys.exit(0)
        
    elif key == ord('a') and not state_manager.drone_armed:  # Arm
        print("Arming drone...")
        control_sequences.arm_drone()
        
    elif key == ord('f'):  # Follow
        if state_manager.drone_armed:
            if not state_manager.altitude_hold:
                print("Starting altitude hold mode (takeoff)...")
                control_sequences.start_altitude_hold()
            
            print("Activating follow mode...")
            state_manager.follow_mode = True
            state_manager.INITIALIZED = False  # Reset target to acquire new one
        else:
            print("Cannot start follow mode: Drone not armed.")
            
    elif key == ord('h'):  # Hover
        print("Emergency hover command")
        movement.send_hover_command()
        
    elif key == ord('l'):  # Land
        print("Landing...")
        control_sequences.land_drone()
        state_manager.drone_armed = False
        state_manager.follow_mode = False
        state_manager.altitude_hold = False
        state_manager.ARMING_SEQ_COMPLETE = False
        
    elif key == ord('d'):  # Diagnostics
        print("Running diagnostics...")
        networking.run_diagnostics()
        
    elif key == ord('r'):  # Reset
        print("Resetting state...")
        state_manager.reset_state()
        
    elif key == ord('t'):  # Toggle test mode
        config.TESTING_MODE = not config.TESTING_MODE
        print(f"Test mode: {'ON' if config.TESTING_MODE else 'OFF'}")
        
    elif key == ord('+') and config.TESTING_MODE:  # Increase safety scale
        config.SAFETY_SCALE = min(config.SAFETY_SCALE + 0.1, 1.0)
        print(f"Safety scaling increased to {config.SAFETY_SCALE*100:.0f}%")
        
    elif key == ord('-') and config.TESTING_MODE:  # Decrease safety scale
        config.SAFETY_SCALE = max(config.SAFETY_SCALE - 0.1, 0.1)
        print(f"Safety scaling decreased to {config.SAFETY_SCALE*100:.0f}%")

# Clean up at the end (this will only be reached if the while loop is broken)
cap.release()
cv2.destroyAllWindows()
networking.close_udp_socket()
