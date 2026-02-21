# supervisor_controller.py
from controller import Supervisor
import numpy as np
import math
import struct

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Get emitter device
emitter = supervisor.getDevice("emitter")

# Robot DEF names
names = ["r1", "r2", "r3", "r4", "r5", "r6"]
robots = [supervisor.getFromDef(name) for name in names]

SENSING_RADIUS = 1

while supervisor.step(timestep) != -1:
    me = supervisor.getSelf()
    my_pos = np.array(me.getPosition())
    neighbor_vectors = []

    for r in robots:
        if r and r.getId() != me.getId():
            pos = np.array(r.getPosition())
            dist = np.linalg.norm(my_pos[:2] - pos[:2])
            if dist <= SENSING_RADIUS:
                vector = pos[:2] - my_pos[:2]
                neighbor_vectors.append(vector)

    if len(neighbor_vectors) > 0:
        avg_vec = np.sum(neighbor_vectors, axis=0) / len(neighbor_vectors)
    else:
        avg_vec = np.array([0.0, 0.0])

    # Pack into bytes (2 floats)
    data = struct.pack("ff", avg_vec[0], avg_vec[1])
    emitter.send(data)