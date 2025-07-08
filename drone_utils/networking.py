# UDP communication utilities for drone control

import socket
import struct
import time
from .config import DRONE_IP, DRONE_PORT, HOVER_THRUST

# Global socket for UDP communication
drone_socket = None

def initialize_udp_socket():
    """Initialize the UDP socket for communication with the drone."""
    global drone_socket
    try:
        if drone_socket is None:
            drone_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            drone_socket.settimeout(0.5)
            print("UDP socket initialized.")
    except Exception as e:
        print(f"Error initializing UDP socket: {e}")
        # Try to close and reinitialize if there was a problem
        try:
            if drone_socket:
                drone_socket.close()
            drone_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            drone_socket.settimeout(0.5)
            print("UDP socket reinitialized after error.")
        except Exception as e2:
            print(f"Failed to reinitialize socket: {e2}")
            return None
    return drone_socket

def close_udp_socket():
    """Close the UDP socket."""
    global drone_socket
    if drone_socket:
        drone_socket.close()
        drone_socket = None
        print("UDP socket closed.")

def test_drone_connection():
    """Test if we can reach the drone via UDP"""
    test_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    test_socket.settimeout(0.5)
    
    # Send a ping packet (neutral joystick command)
    header = 0x30
    test_data = struct.pack('<Bffff', header, 0.0, 0.0, 0.0, 0.0)
    cksum = sum(test_data) % 256
    test_packet = test_data + bytes([cksum])
    
    try:
        test_socket.sendto(test_packet, (DRONE_IP, DRONE_PORT))
        return True
    except Exception as e:
        print(f"Connection test failed: {e}")
        return False
    finally:
        test_socket.close()

def run_diagnostics():
    """Run a diagnostic check of the drone connection and state"""
    import os
    from . import state_manager
    
    print("Running ESP-Drone diagnostics...")
    print(f"Drone IP: {DRONE_IP}")
    print(f"Drone Port: {DRONE_PORT}")
    
    # Check network connectivity
    host_reachable = False
    try:
        ping_result = os.system(f"ping -c 1 -W 1 {DRONE_IP} >/dev/null 2>&1")
        host_reachable = ping_result == 0
    except:
        pass
    
    print(f"Network connection: {'OK' if host_reachable else 'FAILED'}")
    
    # Check state
    print(f"Drone armed: {'YES' if state_manager.drone_armed else 'NO'}")
    print(f"Altitude hold: {'YES' if state_manager.altitude_hold else 'NO'}")
    print(f"Follow mode: {'YES' if state_manager.follow_mode else 'NO'}")
    
    # Make sure socket is initialized
    initialize_udp_socket()
    
    # Send a test packet - using direct socket access to avoid circular import
    header = 0x30
    test_data = struct.pack('<Bffff', header, 0.0, 0.0, 0.0, HOVER_THRUST)
    cksum = sum(test_data) % 256
    test_packet = test_data + bytes([cksum])
    
    try:
        global drone_socket
        if drone_socket:
            drone_socket.sendto(test_packet, (DRONE_IP, DRONE_PORT))
            print(f"Sent PING command")
            return True
    except Exception as e:
        print(f"Command send error: {e}")
        return False
