# Configuration settings and constants for the ESP-Drone control system

import cv2

# Drone UDP connection details
DRONE_IP = '192.168.43.42'
DRONE_PORT = 2390

# ESP32 camera stream URL
ESP32_STREAM_URL = 'http://192.168.43.43/stream'

# Command packets (same as UI_Drone.py)
ARM_PACKET = bytes([0x71, 0x13, 0x33, 0x01])
DISARM_PACKET = bytes([0x71, 0x13, 0x33, 0x00])
TAKEOFF_PACKET = bytes([0x71, 0x13, 0x11, 0x01])
LAND_PACKET = bytes([0x71, 0x13, 0x11, 0x00])

# ESP-Drone protocol constants
HOVER_THRUST = 32767.0    # Mid-point for hovering (float)
TAKEOFF_THRUST = 40000.0  # Higher thrust for takeoff
ARM_TIME = 3.0            # Seconds to wait after arming before takeoff

# Safety and performance settings
TESTING_MODE = True       # Set to True during initial testing
SAFETY_SCALE = 0.3        # Scale all movements to 30% during testing
MAX_COMMANDS_PER_SECOND = 5  # Limit command frequency

# Follow logic thresholds - Enhanced precision
CENTER_TOLERANCE = 20     # Tighter central zone for better precision (was 30)
MICRO_ZONE = 10          # Very fine movement zone for final positioning
AREA_TOLERANCE = 2000    # Tighter tolerance for depth positioning (was 3000)
AREA_MICRO_ZONE = 800    # Fine depth control zone
MOVEMENT_THRESHOLD = 0.03  # Lower threshold for detecting movement (was 0.05)
MAX_CORRECTION_RATE = 0.8  # Maximum intensity for corrections (prevents over-correction)

# PID-inspired control parameters
P_GAIN_POSITION = 1.2    # Position control gain (higher = faster response)
P_GAIN_AREA = 0.8        # Area/depth control gain
DAMPENING = 0.2          # Momentum dampening to reduce oscillation

# Object detection settings
TARGET_CLASS = 0  # 'person' in COCO dataset

# UI settings
FONT = cv2.FONT_HERSHEY_SIMPLEX
RED = (0, 0, 255)
GREEN = (0, 255, 0)
ARROW_COLOR = (255, 255, 0)
