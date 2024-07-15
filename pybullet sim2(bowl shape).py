import pybullet as p
import pybullet_data
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Loading a plain plane as the ground
plane_id = p.loadURDF("plane.urdf")

# Function to generate an inverted half-sphere
def generate_inverted_half_sphere(radius, segments, rings):
    vertices = []
    indices = []

    # Generate vertices
    for i in range(rings + 1):
        theta = i * np.pi / (2 * rings)  # Only half the sphere
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        for j in range(segments + 1):
            phi = j * 2 * np.pi / segments
            sin_phi = np.sin(phi)
            cos_phi = np.cos(phi)
            x = radius * cos_phi * sin_theta
            y = radius * sin_phi * sin_theta
            z = -radius * cos_theta  # Invert the Z-axis
            vertices.append([x, y, z])
            
    # Add center point for the bottom of the bowl
    bottom_center_index = len(vertices)
    vertices.append([0, 0, -radius])

    # Generate indices for faces
    for i in range(rings):
        for j in range(segments):
            first = i * (segments + 1) + j
            second = first + segments + 1
            next_first = first + 1
            next_second = second + 1
            if next_first % (segments + 1) == 0:
                next_first -= (segments + 1)
            if next_second % (segments + 1) == 0:
                next_second -= (segments + 1)
            indices.append([first, next_first, next_second])
            indices.append([first, next_second, second])

    # Generate indices for the bottom center point
    last_ring_start = rings * (segments + 1)
    for j in range(segments):
        first = last_ring_start + j
        second = last_ring_start + (j + 1) % segments
        indices.append([first, second, bottom_center_index])

    return vertices, indices

# Generate the vertices and indices for an inverted half-sphere
vertices, indices = generate_inverted_half_sphere(radius=1, segments=32, rings=16)

# Create a collision shape using the inverted half-sphere
collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    meshScale=[1, 1, 1],
    vertices=vertices,
    indices=[item for sublist in indices for item in sublist]  # Flatten indices list
)

# Create a visual shape using the inverted half-sphere
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    meshScale=[1, 1, 1],
    vertices=vertices,
    indices=[item for sublist in indices for item in sublist],  # Flatten indices list
    rgbaColor=[0, 1, 0, 0.5],
    specularColor=[0.4, 0.4, 0]
)

# Create the bowl
bowl_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=collision_shape_id,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=[0, 0, 0.5]  # Adjust as needed
)

# Creating the golf ball
ball_visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=0.05,
    rgbaColor=[1, 0, 0, 1],
    specularColor=[0.4, 0.4, 0]
)

ball_collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_SPHERE,
    radius=0.05
)

ball_id = p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=ball_collision_shape_id,
    baseVisualShapeIndex=ball_visual_shape_id,
    basePosition=[0.5, 0.5, 2]  # Drop from a position above and to the side of the bowl
)

# Setting gravity
p.setGravity(0, 0, -9.81)

# Run the simulation
for _ in range(2400):  # Run for 10 seconds (240 steps per second)
    p.stepSimulation()
    time.sleep(1/240)

time.sleep(5)

# Disconnect from PyBullet
p.disconnect()
