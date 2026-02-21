"""
Course: Multi-Robot Systems (MRS)
Project: Leader-Follower Formation Control
Description: Controller for the Follower robot.
             Uses P-Control with boost logic to maintain distance from the Leader.
"""

from controller import Robot
import math

# Initialize the Robot instance
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize Devices (Receiver, GPS, Compass)
receiver = robot.getDevice('receiver')
receiver.enable(timestep)

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

# Initialize Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# --- Control Parameters (Aggressive Tuning) ---
K_p = 1.0          # Linear velocity gain (Aggressive response)
K_a = 1.0          # Angular velocity gain (Rapid orientation adjustment)
DESIRED_DIST = 0.05 # Target distance to maintain (meters)
MAX_SPEED = 6.28    # Physical speed limit for e-puck motors

# Main Control Loop
while robot.step(timestep) != -1:
    # Check for incoming messages from the Leader
    if receiver.getQueueLength() > 0:
        message = receiver.getString()
        receiver.nextPacket()
        
        try:
            # Parse the received coordinates
            target_x, target_y = map(float, message.split(":"))
            
            # Get current state: Position and Orientation
            curr_pos = gps.getValues()
            c_val = compass.getValues()
            curr_rad = math.atan2(c_val[0], c_val[1])
            
            # Calculate Euclidean distance and bearing to target
            dist = math.sqrt((target_x - curr_pos[0])**2 + (target_y - curr_pos[1])**2)
            bearing_to_target = math.atan2(target_y - curr_pos[1], target_x - curr_pos[0])
            
            # Calculate angle error (alpha) and normalize to [-pi, pi]
            alpha = (bearing_to_target - curr_rad + math.pi) % (2 * math.pi) - math.pi

            # --- Aggressive Velocity Control Logic ---
            dist_error = dist - DESIRED_DIST
            
            # 1. Calculate Linear Velocity (v)
            v = K_p * dist_error
            
            # Boost Logic: Ensure minimum speed if distance error is significant (> 5cm)
            if dist_error > 0.05:
                v = max(v, 3.5) 
            
            # 2. Calculate Angular Velocity (w)
            w = K_a * alpha

            # 3. Differential Drive Kinematics (Calculate wheel speeds)
            left_speed = v - w
            right_speed = v + w
            
            # 4. Velocity Saturation: Apply motor limits
            left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
            right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)

            # Set Motor Velocities
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

        except ValueError:
            # Handle potential parsing errors
            continue