# Core drone command functions for sending commands to the ESP-Drone

import struct
import time

from .config import (
    DRONE_IP, DRONE_PORT, MAX_COMMANDS_PER_SECOND,
    TESTING_MODE, SAFETY_SCALE, HOVER_THRUST,
    ARM_PACKET, DISARM_PACKET, TAKEOFF_PACKET, LAND_PACKET
)
from . import state_manager
from . import networking

def send_command(roll, pitch, yaw, thrust, cmd_type="MOVE"):
    """Send command to drone with proper protocol formatting (UI_Drone.py/program.py style)"""
    # Always ensure we have a valid socket
    drone_socket = networking.initialize_udp_socket()
    if not drone_socket:
        print("Error: Could not initialize UDP socket")
        return False
    
    # Limit command frequency
    current_time = time.time()
    if (current_time - state_manager.last_command_time < 1.0/MAX_COMMANDS_PER_SECOND and 
        cmd_type == "MOVE"):
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
        state_manager.last_command_time = time.time()
        return True
    except Exception as e:
        print(f"Command send error: {e}")
        # Try to reinitialize the socket if it failed
        networking.close_udp_socket()
        networking.initialize_udp_socket()
        return False

def send_rc_command(roll, pitch, yaw, thrust):
    """Legacy wrapper for send_command"""
    return send_command(roll, pitch, yaw, thrust, cmd_type="MOVE")

# Utility functions to send specific commands
def send_arm_command():
    """Send ARM command directly"""
    packet = ARM_PACKET + bytes([sum(ARM_PACKET) % 256])
    try:
        # Always ensure we have a valid socket
        drone_socket = networking.initialize_udp_socket()
        if not drone_socket:
            print("Error: Could not initialize UDP socket")
            return False
            
        drone_socket.sendto(packet, (DRONE_IP, DRONE_PORT))
        print("✅ ARM command sent")
        return True
    except Exception as e:
        print(f"❌ Failed to send ARM command: {e}")
        # Try to reinitialize the socket if it failed
        networking.close_udp_socket()
        networking.initialize_udp_socket()
        return False

def send_disarm_command():
    """Send DISARM command directly"""
    packet = DISARM_PACKET + bytes([sum(DISARM_PACKET) % 256])
    try:
        # Always ensure we have a valid socket
        drone_socket = networking.initialize_udp_socket()
        if not drone_socket:
            print("Error: Could not initialize UDP socket")
            return False
            
        drone_socket.sendto(packet, (DRONE_IP, DRONE_PORT))
        print("✅ DISARM command sent")
        return True
    except Exception as e:
        print(f"❌ Failed to send DISARM command: {e}")
        # Try to reinitialize the socket if it failed
        networking.close_udp_socket()
        networking.initialize_udp_socket()
        return False

def send_takeoff_command():
    """Send TAKEOFF command directly"""
    packet = TAKEOFF_PACKET + bytes([sum(TAKEOFF_PACKET) % 256])
    try:
        # Always ensure we have a valid socket
        drone_socket = networking.initialize_udp_socket()
        if not drone_socket:
            print("Error: Could not initialize UDP socket")
            return False
            
        drone_socket.sendto(packet, (DRONE_IP, DRONE_PORT))
        print("🚁 TAKEOFF command sent")
        return True
    except Exception as e:
        print(f"❌ Failed to send TAKEOFF command: {e}")
        # Try to reinitialize the socket if it failed
        networking.close_udp_socket()
        networking.initialize_udp_socket()
        return False

def send_land_command():
    """Send LAND command directly"""
    packet = LAND_PACKET + bytes([sum(LAND_PACKET) % 256])
    try:
        # Always ensure we have a valid socket
        drone_socket = networking.initialize_udp_socket()
        if not drone_socket:
            print("Error: Could not initialize UDP socket")
            return False
            
        drone_socket.sendto(packet, (DRONE_IP, DRONE_PORT))
        print("🛬 LAND command sent")
        return True
    except Exception as e:
        print(f"❌ Failed to send LAND command: {e}")
        # Try to reinitialize the socket if it failed
        networking.close_udp_socket()
        networking.initialize_udp_socket()
        return False
