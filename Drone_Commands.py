import cv2
import numpy as np
from ultralytics import YOLO
import time
import socket
import struct
import sys
import os

print(f"ESP-Drone Human Follower - Starting {time.strftime('%Y-%m-%d %H:%M:%S')}")

# Load YOLOv8 model (use yolov8n for speed)
try:
    model = YOLO("yolov8n.pt")
    print("YOLO model loaded successfully")
except Exception as e:
    print(f"Error loading YOLO model: {e}")
    sys.exit(1)

# ESP32 camera stream URL
esp32_stream_url = 'http://192.168.43.43/stream'

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
DRONE_IP = '192.168.43.42'
DRONE_PORT = 2390
print(f"Drone configured at {DRONE_IP}:{DRONE_PORT}")

drone_socket = None

def initialize_udp_socket():
    global drone_socket
    if drone_socket is None:
        drone_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        drone_socket.settimeout(0.5)
        print("UDP socket initialized.")

def close_udp_socket():
    global drone_socket
    if drone_socket:
        drone_socket.close()
        drone_socket = None
        print("UDP socket closed.")

# Constants for logic
TARGET_CLASS = 0  # 'person' in COCO
INITIALIZED = False
follow_mode = False
target_id = None
initial_box = None
initial_area = None

# Drone state tracking
drone_armed = False       # Whether the drone has been armed
altitude_hold = False     # Whether altitude hold mode is active
takeoff_complete = False  # Whether initial takeoff has completed

# Safety controls for testing
TESTING_MODE = True       # Set to True during initial testing
SAFETY_SCALE = 0.3        # Scale all movements to 30% during testing
MAX_COMMANDS_PER_SECOND = 5  # Limit command frequency

# Command packets (same as UI_Drone.py)
ARM_PACKET = bytes([0x71, 0x13, 0x33, 0x01])
DISARM_PACKET = bytes([0x71, 0x13, 0x33, 0x00])
TAKEOFF_PACKET = bytes([0x71, 0x13, 0x11, 0x01])
LAND_PACKET = bytes([0x71, 0x13, 0x11, 0x00])

# ESP-Drone protocol constants
HOVER_THRUST = 32767.0    # Mid-point for hovering (float)
TAKEOFF_THRUST = 40000.0  # Higher thrust for takeoff
ARM_TIME = 3.0            # Seconds to wait after arming before takeoff
ARMING_SEQ_COMPLETE = False  # Flag to track arming sequence

# Last command time to manage frequency
last_command_time = 0

font = cv2.FONT_HERSHEY_SIMPLEX

# Follow logic thresholds - Enhanced precision
CENTER_TOLERANCE = 20  # Tighter central zone for better precision (was 30)
MICRO_ZONE = 10        # Very fine movement zone for final positioning
AREA_TOLERANCE = 2000  # Tighter tolerance for depth positioning (was 3000)
AREA_MICRO_ZONE = 800  # Fine depth control zone
MOVEMENT_THRESHOLD = 0.03  # Lower threshold for detecting movement (was 0.05)
MAX_CORRECTION_RATE = 0.8  # Maximum intensity for corrections (prevents over-correction)

# PID-inspired control parameters
P_GAIN_POSITION = 1.2    # Position control gain (higher = faster response)
P_GAIN_AREA = 0.8        # Area/depth control gain
DAMPENING = 0.2          # Momentum dampening to reduce oscillation

# Dot colors and size
RED = (0, 0, 255)
GREEN = (0, 255, 0)
ARROW_COLOR = (255, 255, 0)

# Enhanced command functions for ESP-Drone

# Send arming command to drone
def arm_drone():
    """Send specific arming sequence to the drone (matching UI_Drone.py approach)"""
    global drone_armed, last_command_time
    
    print("Sending ARM command to drone...")
    
    # 1. First send zero commands to initialize
    for i in range(3):
        send_command(0.0, 0.0, 0.0, 0.0, cmd_type="INIT")
        time.sleep(0.1)
    
    # 2. Send the ARM command using the ARM_PACKET format
    packet = ARM_PACKET + bytes([sum(ARM_PACKET) % 256])
    try:
        initialize_udp_socket()
        drone_socket.sendto(packet, (DRONE_IP, DRONE_PORT))
        print("✅ ARM command sent")
    except Exception as e:
        print(f"❌ Failed to send ARM command: {e}")
        return False
    
    time.sleep(0.3)
    
    # 3. Send a sequence of hover commands with increasing thrust to spin motors
    send_command(0.0, 0.0, 0.0, HOVER_THRUST * 0.8, cmd_type="HOVER")
    time.sleep(0.2)
    send_command(0.0, 0.0, 0.0, HOVER_THRUST, cmd_type="HOVER")
    time.sleep(0.2)
    send_command(0.0, 0.0, 0.0, HOVER_THRUST * 1.2, cmd_type="HOVER")
    time.sleep(0.2)
    send_command(0.0, 0.0, 0.0, HOVER_THRUST, cmd_type="HOVER")
    
    print("Arming sequence completed")
    drone_armed = True
    last_command_time = time.time()
    return True

# Initiate takeoff and altitude hold mode
def start_altitude_hold():
    """Perform takeoff and switch drone to altitude hold mode"""
    global altitude_hold, last_command_time
    
    print("Initiating takeoff sequence...")
    
    # First send the TAKEOFF command using the direct packet format
    send_takeoff_command()
    time.sleep(0.5)
    
    # Step 1: Start with a moderate thrust to begin ascent
    send_command(0.0, 0.0, 0.0, HOVER_THRUST + 5000, cmd_type="TAKEOFF")
    time.sleep(0.3)
    
    # Step 2: Gradually increase thrust for initial lift
    send_command(0.0, 0.0, 0.0, TAKEOFF_THRUST, cmd_type="TAKEOFF")
    time.sleep(0.5)
    
    # Step 3: Maintain higher thrust briefly to achieve desired height
    send_command(0.0, 0.0, 0.0, TAKEOFF_THRUST, cmd_type="TAKEOFF")
    time.sleep(0.5)
    
    # Step 4: Return to neutral hover thrust for stability
    send_command(0.0, 0.0, 0.0, HOVER_THRUST, cmd_type="HOVER")
    time.sleep(0.5)
    
    # Step 5: Fine-tune to stable hover
    send_command(0.0, 0.0, 0.0, HOVER_THRUST, cmd_type="HOVER")
    
    print("Takeoff complete - Altitude hold active")
    altitude_hold = True
    last_command_time = time.time()
    return True

# Send command to drone with unified protocol
def send_command(roll, pitch, yaw, thrust, cmd_type="MOVE"):
    """Send command to drone with proper protocol formatting (UI_Drone.py/program.py style)"""
    global last_command_time
    initialize_udp_socket()

    # Limit command frequency
    current_time = time.time()
    if current_time - last_command_time < 1.0/MAX_COMMANDS_PER_SECOND and cmd_type == "MOVE":
        return

    # Apply safety scaling during testing for movement commands
    if TESTING_MODE and cmd_type == "MOVE":
        roll *= SAFETY_SCALE
        pitch *= SAFETY_SCALE
        yaw *= SAFETY_SCALE
        # For thrust, we scale only the difference from hover point
        if thrust != 0:  # Don't scale 0 thrust (used for arming)
            thrust = HOVER_THRUST + ((thrust - HOVER_THRUST) * SAFETY_SCALE)
            
    # Use the joystick command format with header byte 0x30 (like UI_Drone.py)
    header = 0x30
    payload = struct.pack('<Bffff', header, roll, pitch, yaw, thrust)
    checksum = sum(payload) % 256
    packet = payload + bytes([checksum])

    try:
        drone_socket.sendto(packet, (DRONE_IP, DRONE_PORT))
        print(f"Sent {cmd_type}: roll={roll:.1f}°, pitch={pitch:.1f}°, yaw={yaw:.1f}°/s, thrust={int(thrust)}")
        last_command_time = time.time()
        return True
    except Exception as e:
        print(f"Command send error: {e}")
        return False

# Utility functions to send specific commands (like UI_Drone.py)
def send_arm_command():
    """Send ARM command directly"""
    packet = ARM_PACKET + bytes([sum(ARM_PACKET) % 256])
    try:
        initialize_udp_socket()
        drone_socket.sendto(packet, (DRONE_IP, DRONE_PORT))
        print("✅ ARM command sent")
        return True
    except Exception as e:
        print(f"❌ Failed to send ARM command: {e}")
        return False

def send_disarm_command():
    """Send DISARM command directly"""
    packet = DISARM_PACKET + bytes([sum(DISARM_PACKET) % 256])
    try:
        initialize_udp_socket()
        drone_socket.sendto(packet, (DRONE_IP, DRONE_PORT))
        print("✅ DISARM command sent")
        return True
    except Exception as e:
        print(f"❌ Failed to send DISARM command: {e}")
        return False

def send_takeoff_command():
    """Send TAKEOFF command directly"""
    packet = TAKEOFF_PACKET + bytes([sum(TAKEOFF_PACKET) % 256])
    try:
        initialize_udp_socket()
        drone_socket.sendto(packet, (DRONE_IP, DRONE_PORT))
        print("🚁 TAKEOFF command sent")
        return True
    except Exception as e:
        print(f"❌ Failed to send TAKEOFF command: {e}")
        return False

def send_land_command():
    """Send LAND command directly"""
    packet = LAND_PACKET + bytes([sum(LAND_PACKET) % 256])
    try:
        initialize_udp_socket()
        drone_socket.sendto(packet, (DRONE_IP, DRONE_PORT))
        print("🛬 LAND command sent")
        return True
    except Exception as e:
        print(f"❌ Failed to send LAND command: {e}")
        return False
        
# Backward compatibility wrapper
def send_rc_command(roll, pitch, yaw, thrust):
    """Legacy wrapper for send_command"""
    return send_command(roll, pitch, yaw, thrust, cmd_type="MOVE")

# Map movement command to RPYT values
def movement_to_rc(command, intensity):
    # Default neutral values
    roll = pitch = yaw = 0.0
    thrust = 32767.0  # Mid-point value for hovering
    
    # Scale factors based on ESP-Drone's expected ranges
    roll_pitch_scale = 30.0  # Max angle in degrees
    yaw_scale = 200.0  # Yaw rate in degrees/second
    thrust_scale = 10000.0  # Thrust adjustment range
    
    if command == 'left':
        roll = -roll_pitch_scale * intensity  # Negative roll = left
    elif command == 'right':
        roll = roll_pitch_scale * intensity   # Positive roll = right
    elif command == 'up':
        thrust = 32767.0 + (thrust_scale * intensity)  # Increase thrust to go up
    elif command == 'down':
        thrust = 32767.0 - (thrust_scale * intensity)  # Decrease thrust to go down
    elif command == 'forward':
        pitch = -roll_pitch_scale * intensity  # Note: this might be inverted depending on drone orientation
    elif command == 'backward':
        pitch = roll_pitch_scale * intensity   # Note: this might be inverted depending on drone orientation
    elif command == 'hover':
        roll = pitch = yaw = 0.0
        thrust = 32767.0  # Mid-point for hovering
    
    # Ensure values are within valid ranges
    roll = max(min(roll, roll_pitch_scale), -roll_pitch_scale)
    pitch = max(min(pitch, roll_pitch_scale), -roll_pitch_scale)
    yaw = max(min(yaw, yaw_scale), -yaw_scale)
    thrust = max(min(thrust, 60000.0), 0.0)  # Limit thrust to valid range
    
    return roll, pitch, yaw, thrust

# Combine multiple movement commands into a single RC command with improved blending
def combine_movements(commands, intensities):
    # Default neutral values
    roll = pitch = yaw = 0.0
    thrust = 32767.0  # Mid-point for hovering
    
    # Track movement types for improved blending
    has_horizontal = False
    has_vertical = False
    has_depth = False
    
    # First pass - analyze movement types
    for cmd in commands:
        cmd = cmd.lower()
        if cmd in ['left', 'right']:
            has_horizontal = True
        elif cmd in ['up', 'down']:
            has_vertical = True
        elif cmd in ['forward', 'backward']:
            has_depth = True
    
    # Multi-axis movement scaling factor (reduce intensity when moving in multiple axes)
    axis_count = sum([has_horizontal, has_vertical, has_depth])
    multi_axis_factor = 1.0 if axis_count <= 1 else (1.0 - (0.1 * (axis_count - 1)))
    
    # Second pass - apply movements with coordinated blending
    for cmd, intensity in zip(commands, intensities):
        r, p, y, t = movement_to_rc(cmd.lower(), intensity * multi_axis_factor)
        
        # Apply dampening to reduce oscillation around center point
        if abs(r) < 5.0:  # Small roll corrections
            r *= (1.0 - DAMPENING)
        if abs(p) < 5.0:  # Small pitch corrections
            p *= (1.0 - DAMPENING)
            
        # Combine movements with intelligent blending
        roll += r
        pitch += p
        yaw += y
        
        # For thrust, we use a smarter approach to prevent excessive altitude changes
        if cmd.lower() == 'up':
            thrust = max(thrust, t)
        elif cmd.lower() == 'down':
            thrust = min(thrust, t)
    
    # Ensure values are within valid ranges
    roll_pitch_scale = 30.0  # Max angle in degrees
    yaw_scale = 200.0  # Yaw rate in degrees/second
    roll = max(min(roll, roll_pitch_scale), -roll_pitch_scale)
    pitch = max(min(pitch, roll_pitch_scale), -roll_pitch_scale)
    yaw = max(min(yaw, yaw_scale), -yaw_scale)
    thrust = max(min(thrust, 60000.0), 0.0)  # Limit thrust to valid range
    
    return roll, pitch, yaw, thrust

# Initial arming time tracking
arm_start_time = None
takeoff_delay = 2.0  # Seconds to wait between steps

# Setup startup sequence
print("\n========== ESP-DRONE HUMAN FOLLOWER ==========")
print("Drone control ready. Press 'a' to arm, 'f' to takeoff and follow")
print("\nKey commands:")
print("  'a' - Arm drone (does not takeoff yet)")
print("  'f' - Takeoff and start follow mode (after arming)")
print("  'h' - Emergency hover (stabilize at current position)")
print("  'l' - Land (gradual descent)")
print("  'd' - Run diagnostics (check drone connection)")
print("  'r' - Reset state (if something goes wrong)")
print("  't' - Toggle test mode (currently: {})".format("ON" if TESTING_MODE else "OFF"))
print("  '+/-' - Adjust safety scaling (currently: {}%)".format(int(SAFETY_SCALE*100)))
print("  'q' - Quit")
print("==============================================\n")

# Test network connectivity to drone
def test_drone_connection():
    """Test if we can reach the drone via UDP"""
    test_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    test_socket.settimeout(0.5)
    
    print(f"Testing connection to ESP-Drone at {DRONE_IP}:{DRONE_PORT}...")
    
    # Send a ping packet (neutral joystick command) with proper header (0x30)
    header = 0x30
    test_data = struct.pack('<Bffff', header, 0.0, 0.0, 0.0, 0.0)
    cksum = sum(test_data) % 256
    test_packet = test_data + bytes([cksum])
    
    try:
        test_socket.sendto(test_packet, (DRONE_IP, DRONE_PORT))
        # UDP doesn't guarantee a response, so we'll just assume success if no error
        print("✅ Connection test packet sent successfully")
        return True
    except Exception as e:
        print(f"❌ Connection test failed: {e}")
        return False
    finally:
        test_socket.close()

# Try to verify drone connection at startup
drone_connection_verified = test_drone_connection()
if not drone_connection_verified:
    print("⚠️ WARNING: Could not verify connection to drone!")
    print("Check that the drone is powered on and WiFi is connected.")
    print("Will continue anyway - commands may not reach the drone.")

# Run a diagnostic check of the drone connection and state
def run_diagnostics():
    """Run a diagnostic check of the drone connection and state"""
    print("\n========== ESP-DRONE DIAGNOSTICS ==========")
    print(f"Drone IP: {DRONE_IP}")
    print(f"Drone Port: {DRONE_PORT}")
    
    # Check network connectivity
    host_reachable = False
    try:
        # Try to ping the host (this works on macOS)
        ping_result = os.system(f"ping -c 1 -W 1 {DRONE_IP} >/dev/null 2>&1")
        host_reachable = ping_result == 0
    except:
        pass
    
    print(f"Host reachable: {'✅ YES' if host_reachable else '❌ NO'}")
    
    # Check socket state
    print(f"Socket created: {'✅ YES' if drone_socket else '❌ NO'}")
    
    # Check state variables
    print(f"Drone armed: {'✅ YES' if drone_armed else '❌ NO'}")
    print(f"Arming sequence complete: {'✅ YES' if ARMING_SEQ_COMPLETE else '❌ NO'}")
    print(f"Altitude hold active: {'✅ YES' if altitude_hold else '❌ NO'}")
    print(f"Follow mode: {'✅ YES' if follow_mode else '❌ NO'}")
    
    # Send a ping command and verify
    print("\nSending test packet...")
    ping_success = False
    try:
        # Send a neutral hover command
        ping_success = send_command(0.0, 0.0, 0.0, HOVER_THRUST, cmd_type="PING")
    except Exception as e:
        print(f"Error during test packet: {e}")
    
    print(f"Test packet sent: {'✅ YES' if ping_success else '❌ NO'}")
    print("=========================================\n")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Failed to get frame, retrying...")
        time.sleep(0.5)
        continue

    start_time = time.time()
    frame_resized = cv2.resize(frame, (640, 480))
    h, w, _ = frame_resized.shape
    frame_center = (w // 2, h // 2)

    # Process arming sequence if needed
    if arm_start_time and not ARMING_SEQ_COMPLETE:
        time_since_arm = time.time() - arm_start_time
        # Visual indicator of arming sequence
        progress = min(time_since_arm / ARM_TIME, 1.0)
        bar_width = int(w * 0.6)
        bar_progress = int(bar_width * progress)
        cv2.rectangle(frame_resized, (w//2 - bar_width//2, h//2 - 30), 
                     (w//2 - bar_width//2 + bar_progress, h//2 - 20), 
                     (0, 255, 0), -1)
        cv2.rectangle(frame_resized, (w//2 - bar_width//2, h//2 - 30), 
                     (w//2 + bar_width//2, h//2 - 20), 
                     (255, 255, 255), 2)
        cv2.putText(frame_resized, "ARMING SEQUENCE", (w//2 - 100, h//2 - 40), 
                   font, 0.7, (0, 255, 255), 2)
        
        # Complete arming sequence but don't take off yet
        if time_since_arm >= ARM_TIME:
            ARMING_SEQ_COMPLETE = True
            print("Drone armed and ready! Press 'f' to begin takeoff and following.")

    key = cv2.waitKey(1) & 0xFF
    if key == ord('a'):  # Arm drone
        if not drone_armed:
            print("Starting drone arm sequence...")
            # UI_Drone.py style arming
            arm_drone()
            arm_start_time = time.time()
        else:
            print("Drone is already armed")
    elif key == ord('f'):  # Start follow mode
        if drone_armed and ARMING_SEQ_COMPLETE:
            if not takeoff_complete:
                print("Starting takeoff sequence...")
                start_altitude_hold()
                takeoff_complete = True
                time.sleep(1.0)  # Give the drone a moment to stabilize
                print("Takeoff complete - starting follow mode")
            
            follow_mode = True
            INITIALIZED = False
            print("Follow mode enabled - tracking target")
        else:
            print("Cannot start follow mode: Drone must be armed first!")
            print("Press 'a' to arm the drone")
    elif key == ord('l'):  # Land
        print("LANDING: Initiating descent sequence...")
        
        # First send the LAND command (matches UI_Drone.py approach)
        send_land_command()
        time.sleep(0.5)
        
        # Then send gradual descent commands
        landing_sequence = [
            (30000, 0.4),  # Thrust, delay (seconds)
            (28000, 0.4),
            (26000, 0.4),
            (24000, 0.4),
            (22000, 0.4),
            (20000, 0.5),
            (17000, 0.5),
            (14000, 0.5),
            (10000, 0.5),
            (5000, 0.5),
            (0, 0.5)
        ]
        
        for thrust, delay in landing_sequence:
            send_command(0.0, 0.0, 0.0, float(thrust), cmd_type="LAND")
            time.sleep(delay)
        
        # Final disarm command
        send_disarm_command()
        
        print("Landing complete")
        drone_armed = False
        follow_mode = False
        ARMING_SEQ_COMPLETE = False
    elif key == ord('q'):
        # Land before quitting if armed
        if drone_armed:
            print("Landing before exit...")
            send_command(0.0, 0.0, 0.0, 0.0, cmd_type="LAND")
            time.sleep(1.0)
        break
    elif key == ord('t'):  # Toggle testing mode
        TESTING_MODE = not TESTING_MODE
        print(f"Testing mode: {'ON' if TESTING_MODE else 'OFF'}")
    elif key == ord('+') or key == ord('='):  # Increase safety scale
        SAFETY_SCALE = min(SAFETY_SCALE + 0.1, 1.0)
        print(f"Safety scale: {SAFETY_SCALE:.1f}")
    elif key == ord('-'):  # Decrease safety scale
        SAFETY_SCALE = max(SAFETY_SCALE - 0.1, 0.1)
        print(f"Safety scale: {SAFETY_SCALE:.1f}")
    elif key == ord('h'):  # Emergency hover
        send_command(0.0, 0.0, 0.0, HOVER_THRUST, cmd_type="HOVER")
        print("EMERGENCY HOVER COMMAND SENT")
    elif key == ord('d'):  # Run diagnostics
        run_diagnostics()
    elif key == ord('r'):  # Reset state
        print("Resetting drone state...")
        drone_armed = False
        follow_mode = False
        ARMING_SEQ_COMPLETE = False
        arm_start_time = None
        print("State reset - press 'a' to arm drone again")

    # Status indicators
    status_text = []
    if TESTING_MODE:
        status_text.append(f"TEST MODE ({SAFETY_SCALE*100:.0f}%)")
    
    # Drone state indicators
    status_color = (0, 0, 255)  # Default red = not ready
    
    if not drone_armed:
        status_text.append("NOT ARMED (Press 'a' to arm)")
    elif not ARMING_SEQ_COMPLETE:
        status_text.append("ARMING IN PROGRESS")
        status_color = (0, 165, 255)  # Orange
    elif not takeoff_complete:
        status_text.append("ARMED & READY (Press 'f' to takeoff & follow)")
        status_color = (0, 255, 255)  # Yellow
    elif not follow_mode:
        status_text.append("HOVERING (Press 'f' to follow)")
        status_color = (0, 255, 0)  # Green
    else:
        status_text.append("FOLLOWING TARGET")
        status_color = (0, 255, 0)  # Green
    
    # Draw status indicator dot
    cv2.circle(frame_resized, (20, 60), 10, status_color, -1)
    
    # Put status text
    cv2.putText(frame_resized, " | ".join(status_text), (40, 65), font, 0.6, status_color, 2)
    
    if not follow_mode:
        cv2.imshow("Drone View", frame_resized)
        continue

    results = model.predict(frame_resized, conf=0.5, iou=0.5, classes=[TARGET_CLASS], verbose=False)
    detections = results[0].boxes.data.cpu().numpy()

    if not INITIALIZED:
        if len(detections) > 0:
            # Pick the largest person detected
            target_box = max(detections, key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))
            x1, y1, x2, y2, conf, cls = target_box[:6]
            target_id = 1  # Simulated ID
            initial_box = (int(x1), int(y1), int(x2), int(y2))
            initial_area = (x2 - x1) * (y2 - y1)
            INITIALIZED = True
            print("[INFO] Human locked.")
    else:
        # Try to track the same human
        best_match = None
        min_dist = float('inf')
        for det in detections:
            x1, y1, x2, y2, conf, cls = det[:6]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            area = (x2 - x1) * (y2 - y1)

            init_cx = (initial_box[0] + initial_box[2]) // 2
            init_cy = (initial_box[1] + initial_box[3]) // 2
            dist = np.sqrt((init_cx - cx)**2 + (init_cy - cy)**2)

            if dist < min_dist:
                min_dist = dist
                best_match = (int(x1), int(y1), int(x2), int(y2), area, cx, cy)

        if best_match is not None:
            x1, y1, x2, y2, area, cx, cy = best_match
            cv2.rectangle(frame_resized, (x1, y1), (x2, y2), GREEN, 2)
            dot_size = int(15 * np.sqrt(area / initial_area))
            cv2.circle(frame_resized, (cx, cy), dot_size, GREEN, -1)

            # Drone camera center
            cv2.circle(frame_resized, frame_center, 10, RED, -1)
            cv2.line(frame_resized, frame_center, (cx, cy), ARROW_COLOR, 2)

            # Draw vector line with arrow from center to target
            cv2.arrowedLine(frame_resized, frame_center, (cx, cy), ARROW_COLOR, 2, tipLength=0.2)
            
            # Enhanced visualization with multi-zone targeting
            # Draw outer tolerance zone (standard correction zone)
            cv2.rectangle(frame_resized, 
                        (frame_center[0] - CENTER_TOLERANCE, frame_center[1] - CENTER_TOLERANCE),
                        (frame_center[0] + CENTER_TOLERANCE, frame_center[1] + CENTER_TOLERANCE),
                        (0, 165, 255), 1)
            
            # Draw inner micro-adjustment zone (precision targeting)
            cv2.rectangle(frame_resized, 
                        (frame_center[0] - MICRO_ZONE, frame_center[1] - MICRO_ZONE),
                        (frame_center[0] + MICRO_ZONE, frame_center[1] + MICRO_ZONE),
                        (0, 255, 255), 1)
            
            # Target size indicators
            target_area_text = f"Area: {area:.0f} / {initial_area:.0f}"
            cv2.putText(frame_resized, target_area_text, (cx - 50, cy + 30), font, 0.5, GREEN, 1)

            # Enhanced precision movement logic with multi-zone approach
            dx = cx - frame_center[0]  # Horizontal offset from center
            dy = cy - frame_center[1]  # Vertical offset from center
            da = area - initial_area   # Area difference (proxy for distance)
            
            # Calculate tracking error for visualization and logging
            tracking_error = np.sqrt(dx**2 + dy**2)
            area_error_pct = abs(da) / max(initial_area, 100) * 100
            
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
                area_ratio = abs(da) / max(initial_area, 100)
                # More aggressive scaling for distance adjustments
                intensity = min(area_ratio * 2.0 * P_GAIN_AREA, MAX_CORRECTION_RATE)
                intensities.append(intensity)
            elif abs(da) > AREA_MICRO_ZONE:
                # Fine depth positioning zone
                move = "FORWARD" if da < 0 else "BACKWARD"
                command.append(move)
                # Micro-adjustments for distance
                area_ratio = abs(da) / max(initial_area, 100)
                intensity = min(area_ratio * 1.0 * P_GAIN_AREA * 0.5, 0.3)
                intensities.append(intensity)
                
            # If no corrections needed, hover in place
            if not command:
                command = ["HOVER"]
                intensities = [0.0]
            
            # Generate a single combined command with improved visualization
            if command[0] != "HOVER":
                roll, pitch, yaw, thrust = combine_movements(command, intensities)
                
                # Enhanced visual feedback - show movement vectors with intensity indicators
                vector_length = 30  # Base vector length
                
                # Roll indicator (left/right movement)
                if roll != 0:
                    # Calculate vector length based on intensity
                    roll_intensity = abs(roll) / 30.0  # Normalize by max roll angle
                    roll_vector_len = int(vector_length * min(roll_intensity * 2, 1.5))
                    
                    # Draw arrow with length proportional to movement intensity
                    if roll < 0:  # Left movement
                        start_x = w//2 - 5
                        end_x = max(w//2 - roll_vector_len - 15, 5)
                        cv2.arrowedLine(frame_resized, (start_x, h//2), (end_x, h//2), 
                                      ARROW_COLOR, 2, tipLength=0.3)
                        direction = "L"
                    else:  # Right movement
                        start_x = w//2 + 5
                        end_x = min(w//2 + roll_vector_len + 15, w-5)
                        cv2.arrowedLine(frame_resized, (start_x, h//2), (end_x, h//2), 
                                      ARROW_COLOR, 2, tipLength=0.3)
                        direction = "R"
                        
                    # Show command strength percentage
                    strength_pct = int(roll_intensity * 100)
                    cv2.putText(frame_resized, f"{direction}:{strength_pct}%", 
                              (w//2 + (30 if roll > 0 else -60), h//2 - 10), 
                              font, 0.6, ARROW_COLOR, 2)
                
                # Pitch indicator (forward/backward movement)
                if pitch != 0:
                    # Calculate vector length based on intensity
                    pitch_intensity = abs(pitch) / 30.0  # Normalize by max pitch angle
                    pitch_vector_len = int(vector_length * min(pitch_intensity * 2, 1.5))
                    
                    # Draw arrow with length proportional to movement intensity
                    if pitch < 0:  # Forward movement
                        start_y = h//2 - 5
                        end_y = max(h//2 - pitch_vector_len - 15, 5)
                        cv2.arrowedLine(frame_resized, (w//2, start_y), (w//2, end_y), 
                                      ARROW_COLOR, 2, tipLength=0.3)
                        direction = "F"
                    else:  # Backward movement
                        start_y = h//2 + 5
                        end_y = min(h//2 + pitch_vector_len + 15, h-5)
                        cv2.arrowedLine(frame_resized, (w//2, start_y), (w//2, end_y), 
                                      ARROW_COLOR, 2, tipLength=0.3)
                        direction = "B"
                        
                    # Show command strength percentage
                    strength_pct = int(pitch_intensity * 100)
                    cv2.putText(frame_resized, f"{direction}:{strength_pct}%", 
                              (w//2 + 10, h//2 + (30 if pitch > 0 else -30)), 
                              font, 0.6, ARROW_COLOR, 2)
                
                # Send the calculated command to the drone
                send_rc_command(roll, pitch, yaw, thrust)
            else:
                # Display "HOVER" indicator when perfectly centered
                cv2.putText(frame_resized, "HOVER", (w//2 - 30, h//2 - 30), font, 0.7, (0, 255, 0), 2)
                send_rc_command(0.0, 0.0, 0.0, 32767.0)  # Hover command

            status_line = f"Command: {' | '.join(command)}"
            if TESTING_MODE:
                status_line = f"TEST ({SAFETY_SCALE*100:.0f}%) | " + status_line
            cv2.putText(frame_resized, status_line, (10, 30), font, 0.7, (0, 255, 255), 2)

            # Enhanced tracking metrics with colored status indicators
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
            
            cv2.putText(frame_resized, position_error, (w-240, h-30), font, 0.6, error_color, 2)
            cv2.putText(frame_resized, area_error_text, (w-240, h-10), font, 0.6, error_color, 2)
            
            # Enhanced distance indicators with directional hints
            dx_text = f"dx: {dx:+.1f}"
            dy_text = f"dy: {dy:+.1f}"
            
            # Display offset values near the target
            cv2.putText(frame_resized, dx_text, (cx + 20, cy), font, 0.5, (255, 255, 0), 1)
            cv2.putText(frame_resized, dy_text, (cx + 20, cy + 15), font, 0.5, (255, 255, 0), 1)
            
            # Draw accuracy circle that shows how close we are to perfect centering
            accuracy_radius = max(5, int(tracking_error / 2))
            cv2.circle(frame_resized, frame_center, accuracy_radius, error_color, 1)
        else:
            cv2.putText(frame_resized, "Target Lost: Hovering", (10, 30), font, 0.7, (0, 0, 255), 2)
            send_rc_command(0.0, 0.0, 0.0, 32767.0)  # Hover at mid-thrust

    end_time = time.time()
    fps = 1 / (end_time - start_time)
    cv2.putText(frame_resized, f"FPS: {fps:.2f}", (500, 30), font, 0.6, (255, 255, 255), 1)

    cv2.imshow("Drone View", frame_resized)

cap.release()
cv2.destroyAllWindows()