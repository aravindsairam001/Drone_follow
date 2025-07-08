# ESP-Drone Human Following System


## Overview

This project enables an ESP32-based drone to autonomously detect, track, and follow human targets using computer vision and real-time motion control. The system uses YOLOv8 for object detection and implements a sophisticated movement control algorithm to keep the human subject centered in the camera view, maintaining an optimal following distance.

The drone can be controlled manually using keyboard commands or set to autonomous follow mode, where it will maintain position relative to the detected person. Visual feedback provides real-time status, tracking metrics, and movement indicators.

## Project Structure

The codebase follows a modular architecture for improved maintainability and extensibility:

```
Drone_follow/
├── main.py                # Main executable script
├── drone_utils/           # Utility modules
│   ├── __init__.py        # Package marker
│   ├── config.py          # Configuration settings and constants
│   ├── commands.py        # Core drone command functions
│   ├── networking.py      # UDP communication utilities
│   ├── state_manager.py   # State management for drone control
│   ├── control_sequences.py # High-level control sequences
│   ├── vision.py          # Object detection and tracking
│   └── movement.py        # Movement control and navigation
├── screenshots/           # Documentation images
└── yolov8n.pt             # YOLOv8 model (nano version)
```

## Core Features

- **Real-time Human Detection & Tracking**: Uses YOLOv8 to detect and track humans in the camera feed
- **Autonomous Following**: Generates movement commands to maintain optimal position relative to the target
- **Multi-zone Control Logic**: Different control sensitivities based on target position
- **Visual Feedback System**: Rich overlay with tracking metrics, status indicators, and movement visualization
- **Intelligent Movement Blending**: Combines multiple movement vectors for smooth control
- **Flexible Control Modes**: Switch between manual control and autonomous following
- **Safety Features**: Test mode with adjustable command intensity, emergency hover, controlled landing
- **Robust Networking**: Reliable UDP communication with error handling and recovery
- **Camera Fallback**: Automatically falls back to webcam if ESP32 camera is unavailable

## Requirements

- Python 3.8+
- OpenCV 4.5+
- NumPy
- Ultralytics YOLOv8
- ESP-Drone with ESP32 camera module
- Network connection to the drone

## Setup

1. Install the required packages:
   ```
   pip install opencv-python numpy ultralytics
   ```

2. Configure the drone and camera settings in `drone_utils/config.py`:
   ```python
   # Example configuration
   DRONE_IP = "192.168.43.42"  # Your drone's IP address
   DRONE_PORT = 2390           # UDP port for drone communication
   ESP32_STREAM_URL = "http://192.168.43.43/stream"  # ESP32 camera URL
   ```

3. Ensure the YOLOv8 model is available in the project root:
   ```
   yolov8n.pt  # Nano version is recommended for better performance
   ```

## Usage

Run the main script to start the application:
```
python main.py
```

The program will automatically:
1. Load the YOLOv8 model
2. Connect to the ESP32 camera (or fall back to webcam)
3. Initialize the UDP socket for drone communication
4. Start the main control loop

### Controls

- `a` - Arm drone (begins arming sequence but does not take off)
- `f` - Start follow mode (performs takeoff if needed, then begins tracking)
- `h` - Emergency hover (immediately stabilizes at current position)
- `l` - Land (executes controlled descent sequence)
- `d` - Run diagnostics (check drone connection status)
- `r` - Reset state (if something goes wrong)
- `t` - Toggle test mode (limits command intensity for safety)
- `+/-` - Adjust safety scaling in test mode (10% to 100%)
- `q` - Quit the application

## Visual Interface

The program provides a rich visual interface with real-time information:

![Tracking Interface](screenshots/tracking_interface.jpg)

### Status Indicators
- **Status Dot**: Green when armed, red when disarmed
- **Mode Display**: Shows current modes (ARMED, ALT HOLD, FOLLOW MODE, TEST)
- **Arming Sequence**: Visual progress bar during arming
- **FPS Counter**: Shows processing performance

### Tracking Visualization
- **Target Bounding Box**: Green box around the tracked human
- **Tracking Metrics**: Distance, size, and position relative to center
- **Movement Arrows**: Directional indicators showing current movement commands
- **Intensity Bars**: Visual representation of command strength
- **Zone Indicators**: Shows which control zone the target is in

## Technical Implementation

### Movement Control Logic

The system uses a sophisticated multi-zone control approach:

- **Outer Zone**: Stronger corrections with higher intensity for large offsets
- **Middle Zone**: Moderate corrections for general positioning
- **Inner Zone**: Fine micro-adjustments for precise positioning
- **Center Zone**: Minimal or no movement when target is correctly positioned

Movement commands include:
- **Roll** (left/right): Keeps target centered horizontally
- **Thrust** (up/down): Maintains vertical positioning
- **Pitch** (forward/backward): Maintains optimal distance based on target size
- **Yaw**: Currently not used, but available for future enhancements

### Command Protocol

The drone uses a joystick-style protocol with the following structure:
1. Header byte (0x30)
2. Roll value (float)
3. Pitch value (float)
4. Yaw value (float)
5. Thrust value (float)
6. Checksum byte

Special commands (ARM, DISARM, TAKEOFF, LAND) use predefined packet formats.

### Networking

Communication with the drone is handled through UDP:
- Socket is initialized at startup
- Commands are rate-limited to prevent flooding
- Comprehensive error handling ensures robustness
- Automatic socket reinitialization if errors occur

### Vision System

The computer vision pipeline includes:
1. Frame capture from ESP32 camera or webcam
2. YOLOv8 object detection to identify humans
3. Target selection based on size and position
4. Tracking through successive frames
5. Position analysis to generate movement commands

## Troubleshooting

- **No camera detected**: Check ESP32 camera power and connectivity
- **Connection failures**: Verify drone IP and port settings
- **Command send errors**: Ensure network connection is stable
- **Low FPS**: Consider using a smaller YOLOv8 model or reducing resolution
- **Erratic movement**: Adjust dampening settings in config.py
- **Socket errors**: The system will automatically try to reinitialize sockets

## Future Enhancements

- Obstacle avoidance using depth estimation
- Multiple target tracking and selection
- Gesture control recognition
- Path planning for smoother navigation
- Video recording and streaming capabilities

## Acknowledgments

- ESP-Drone project for the drone firmware
- Ultralytics for the YOLOv8 object detection model
- OpenCV community for computer vision tools
