# State management for drone control system

# Drone state tracking
drone_armed = False       # Whether the drone has been armed
altitude_hold = False     # Whether altitude hold mode is active
takeoff_complete = False  # Whether initial takeoff has completed
ARMING_SEQ_COMPLETE = False  # Flag to track arming sequence

# Object tracking state
INITIALIZED = False
follow_mode = False
target_id = None
initial_box = None
initial_area = None

# Timing variables
arm_start_time = None
last_command_time = 0
takeoff_delay = 2.0  # Seconds to wait between steps

def reset_state():
    """Reset all drone state variables to default values"""
    global drone_armed, altitude_hold, takeoff_complete
    global ARMING_SEQ_COMPLETE, follow_mode, INITIALIZED
    global arm_start_time, last_command_time
    
    drone_armed = False
    altitude_hold = False
    takeoff_complete = False
    ARMING_SEQ_COMPLETE = False
    follow_mode = False
    INITIALIZED = False
    arm_start_time = None
    
    print("Drone state has been reset.")
