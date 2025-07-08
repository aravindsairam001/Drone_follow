#!/usr/bin/env python3
"""
ESP-Drone UDP Command Protocol Test Script

This script tests sending commands to ESP-Drone using the correct protocol.
The ESP-Drone expects commands as 4 floats (roll, pitch, yaw, thrust) + 1 checksum byte.
"""

import socket
import struct
import time
import sys

# Configuration
DRONE_IP = "192.168.43.42"     # Replace with your drone's IP
DRONE_PORT = 2390            # Replace with the correct CRTP/command port
COMMAND_RATE = 10            # Hz (times per second)
TEST_DURATION = 5            # seconds
HOVER_THRUST = 32767.0       # Mid-point for hovering thrust

# ESP-Drone protocol: 4 floats + checksum byte
def create_esp_drone_packet(roll, pitch, yaw_rate, thrust):
    """
    Creates a packet for ESP-Drone in the proper format:
    - 4 float32 values (roll, pitch, yaw, thrust)
    - 1 byte checksum (sum of all bytes % 256)
    """
    # Pack as 4 floats in little-endian format
    data = struct.pack('<ffff', float(roll), float(pitch), float(yaw_rate), float(thrust))
    
    # Calculate checksum (simple sum of all bytes modulo 256)
    cksum = sum(bytearray(data)) % 256
    
    # Append checksum to data
    packet = data + struct.pack('B', int(cksum))
    
    return packet

# Test command: move forward (positive pitch), moderate thrust
test_command = create_crtp_setpoint_packet(
    roll=0.0,         # No roll
    pitch=10.0,       # Forward pitch
    yaw_rate=0.0,     # No rotation
    thrust=30000      # Moderate thrust (value between 0 and ~60000)
)

# UDP send loop
def send_test_commands():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("🚀 Sending test movement commands to drone...")

    try:
        end_time = time.time() + TEST_DURATION
        while time.time() < end_time:
            sock.sendto(test_command, (DRONE_IP, DRONE_PORT))
            time.sleep(1.0 / COMMAND_RATE)

        print("✅ Test commands sent successfully.")
    except Exception as e:
        print(f"❌ Error sending test commands: {e}")
    finally:
        sock.close()

if __name__ == "__main__":
    send_test_commands()