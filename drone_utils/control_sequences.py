# Control sequences for drone operations (arming, takeoff, landing, etc.)

import time
from . import state_manager
from .commands import send_command, send_arm_command, send_takeoff_command, send_land_command, send_disarm_command
from .config import HOVER_THRUST, TAKEOFF_THRUST, ARM_TIME

def arm_drone():
    """Send specific arming sequence to the drone (matching UI_Drone.py approach)"""
    
    print("Sending ARM command to drone...")
    
    # 1. First send zero commands to initialize
    for i in range(3):
        send_command(0.0, 0.0, 0.0, 0.0, cmd_type="INIT")
        time.sleep(0.1)
    
    # 2. Send the ARM command using the ARM_PACKET format
    success = send_arm_command()
    if not success:
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
    state_manager.drone_armed = True
    state_manager.arm_start_time = time.time()
    return True

def start_altitude_hold():
    """Perform takeoff and switch drone to altitude hold mode"""
    
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
    state_manager.altitude_hold = True
    return True

def land_drone():
    """Execute landing sequence with gradual descent"""
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
    state_manager.drone_armed = False
    state_manager.follow_mode = False
    state_manager.ARMING_SEQ_COMPLETE = False
    return True

def emergency_hover():
    """Send emergency hover command to stabilize the drone"""
    send_command(0.0, 0.0, 0.0, HOVER_THRUST, cmd_type="HOVER")
    print("EMERGENCY HOVER COMMAND SENT")
    return True
