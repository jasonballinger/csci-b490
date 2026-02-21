from controller import Robot
import numpy as np
import math
import random

# setup
robot = Robot()
timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice("gps")
gps.enable(timestep)

compass = robot.getDevice("compass")
compass.enable(timestep)

emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
receiver.enable(timestep)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

SENSING_RADIUS = 0.6
MAX_SPEED = 6.28
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052
KP_HEADING = 2.5
KP_FORWARD = 1.0
STOP_THRESHOLD = 0.05
COHESION_GAIN = 1.0
REPULSION_GAIN = 0.0015

# loop
while robot.step(timestep) != -1:

    # step 1: perception
    my_pos = np.array(gps.getValues()[:2])

    message = f"{my_pos[0]},{my_pos[1]}"
    emitter.send(message.encode("utf-8"))

    neighbor_positions = []
    while receiver.getQueueLength() > 0:
        msg = receiver.getString()
        x_str, y_str = msg.split(",")
        other_pos = np.array([float(x_str), float(y_str)])
        receiver.nextPacket()

        distance = np.linalg.norm(my_pos - other_pos)

        if 0 < distance <= SENSING_RADIUS:
            neighbor_positions.append(other_pos)

    #\ step 2: consensus calculation
    neighbor_positions.append(my_pos)
    avg_position = np.mean(neighbor_positions, axis=0)
    cohesion_vector = avg_position - my_pos
    repulsion_vector = np.array([0.0, 0.0])

    for other_pos in neighbor_positions:
        offset = my_pos - other_pos
        distance = np.linalg.norm(offset)

        if distance > 0:
            repulsion_vector += offset / (distance**2)

        direction = (
            COHESION_GAIN * cohesion_vector +
            REPULSION_GAIN * repulsion_vector
        )
        
    distance_to_target = np.linalg.norm(direction)

    if distance_to_target < STOP_THRESHOLD:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        continue

    desired_heading = math.atan2(direction[1], direction[0])
    compass_values = compass.getValues()
    robot_heading = math.atan2(compass_values[0], compass_values[2])
    heading_error = desired_heading - robot_heading
    
    if abs(heading_error) < 0.05:
        heading_error = 0.0

    while heading_error > math.pi:
        heading_error -= 2 * math.pi
    while heading_error < -math.pi:
        heading_error += 2 * math.pi

    # step 3: actuation
    omega = KP_HEADING * heading_error
    v = KP_FORWARD * distance_to_target * max(math.cos(heading_error), 0)
    right_speed = (2 * v + AXLE_LENGTH * omega) / (2 * WHEEL_RADIUS)
    left_speed  = (2 * v - AXLE_LENGTH * omega) / (2 * WHEEL_RADIUS)
    right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
    left_speed  = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)