"""
Course: Multi-Robot Systems (MRS)
Project: Leader-Follower Formation Control
Description: Controller for the Leader robot. 
             Broadcasts GPS coordinates via Emitter.
"""

from controller import Robot

# Initialize the Robot instance
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize Communication (Emitter)
emitter = robot.getDevice('emitter')

# Initialize Sensing (GPS)
gps = robot.getDevice('gps')
gps.enable(timestep)

# Initialize Actuators (Motors)
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Main Control Loop
while robot.step(timestep) != -1:
    # Get current GPS coordinates
    pos = gps.getValues()
    
    # Pack coordinates into a string message "x:y"
    message = f"{pos[0]}:{pos[1]}" 
    
    # Broadcast the message to all followers (UTF-8 encoding)
    emitter.send(message.encode('utf-8'))
    
    # Movement: Circular trajectory
    left_motor.setVelocity(3.0)
    right_motor.setVelocity(4.0)