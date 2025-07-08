"""
Movement control functions for drone positioning
"""

from .config import HOVER_THRUST, DAMPENING
from .commands import send_rc_command

def movement_to_rc(command, intensity):
    """
    Map movement command to RPYT (Roll, Pitch, Yaw, Thrust) values
    
    Args:
        command (str): Movement direction ('left', 'right', 'up', 'down', 'forward', 'backward', 'hover')
        intensity (float): Command intensity value between 0 and 1
        
    Returns:
        tuple: (roll, pitch, yaw, thrust) values calculated for the command
    """
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

def combine_movements(commands, intensities):
    """
    Combine multiple movement commands into a single RC command with improved blending
    
    Args:
        commands (list): List of movement directions
        intensities (list): List of corresponding intensities
        
    Returns:
        tuple: Combined (roll, pitch, yaw, thrust) values
    """
    # Default neutral values
    roll = pitch = yaw = 0.0
    thrust = HOVER_THRUST  # Mid-point for hovering
    
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

def send_hover_command():
    """Send a hover command to the drone"""
    return send_rc_command(0.0, 0.0, 0.0, HOVER_THRUST)

def execute_movement(command, intensities):
    """
    Execute a movement command by sending RPYT values to the drone
    
    Args:
        command (list): List of movement commands
        intensities (list): List of corresponding intensities
        
    Returns:
        bool: Success of the command
    """
    if not command:
        return send_hover_command()
        
    if command[0] == "HOVER":
        return send_hover_command()
    else:
        roll, pitch, yaw, thrust = combine_movements(command, intensities)
        return send_rc_command(roll, pitch, yaw, thrust)
