import pybullet as p
import pybullet_data
import time
import numpy as np
from noise import pnoise2
import random

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a plain plane as the ground
plane_id = p.loadURDF("plane.urdf")

# Function to generate a Perlin noise heightmap
def generate_perlin_noise_heightmap(width, height, scale, octaves, persistence, lacunarity):
    heightmap = np.zeros((width, height))
    for i in range(width):
        for j in range(height):
            heightmap[i][j] = pnoise2(i / scale,
                                      j / scale,
                                      octaves=octaves,
                                      persistence=persistence,
                                      lacunarity=lacunarity)
    return heightmap

# Function to generate vertices and indices based on heightmap
def generate_random_green(width, height, scale, octaves, persistence, lacunarity, height_multiplier):
    heightmap = generate_perlin_noise_heightmap(width, height, scale, octaves, persistence, lacunarity)
    vertices = []
    indices = []

    for i in range(width):
        for j in range(height):
            vertices.append([i, j, heightmap[i][j] * height_multiplier])

    for i in range(width - 1):
        for j in range(height - 1):
            indices.append([i * height + j, (i + 1) * height + j, i * height + j + 1])
            indices.append([(i + 1) * height + j, (i + 1) * height + j + 1, i * height + j + 1])

    return vertices, indices, heightmap

# Parameters for the Perlin noise
width = 100
height = 100
scale = 30.0
octaves = 4
persistence = 0.5
lacunarity = 2.0
height_multiplier = 5.0  # Increased height multiplier

# Generate the vertices and indices for the random green
vertices, indices, heightmap = generate_random_green(width, height, scale, octaves, persistence, lacunarity, height_multiplier)

# Create a collision shape using the random green
collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    meshScale=[1, 1, 1],
    vertices=vertices,
    indices=[item for sublist in indices for item in sublist]  # Flatten indices list
)

# Create a visual shape using the random green
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    meshScale=[1, 1, 1],
    vertices=vertices,
    indices=[item for sublist in indices for item in sublist],  # Flatten indices list
    rgbaColor=[0, 1, 0, 1],  # Opaque color for better visibility
    specularColor=[0.4, 0.4, 0]
)

# Create the green
green_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=collision_shape_id,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=[-width / 2, -height / 2, 0]  # Center the green
)

# Creating the golf ball
ball_visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=0.05,  # Reduced ball size to 0.05
    rgbaColor=[1, 0, 0, 1],  # Opaque red color for better visibility
    specularColor=[0.4, 0.4, 0]
)

ball_collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_SPHERE,
    radius=0.05  # Reduced ball size to 0.05
)

# Determine the maximum height of the green to position the ball above it
max_height = np.max(heightmap) * height_multiplier

# Drop the ball at the center of the green, slightly above the highest point
ball_position = [0, 0, max_height + 5]
ball_id = p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=ball_collision_shape_id,
    baseVisualShapeIndex=ball_visual_shape_id,
    basePosition=ball_position
)

# Print ball details for debugging
print(f"Ball ID: {ball_id}")
print(f"Ball initial position: {ball_position}")

# Check if ball is created and visible
if ball_id >= 0:
    print("Ball created successfully.")
else:
    print("Failed to create ball.")

# Adding the golf hole
hole_position = [10, 10, np.min(heightmap) * height_multiplier]  # Place the hole at a specific position
hole_radius = 0.05  # Same radius as the ball
hole_height = 1.0

hole_visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_CYLINDER,
    radius=hole_radius,
    length=hole_height,
    rgbaColor=[0, 0, 0, 1],  # Black color for the hole
    specularColor=[0.4, 0.4, 0]
)

hole_collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_CYLINDER,
    radius=hole_radius,
    height=hole_height
)

hole_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=hole_collision_shape_id,
    baseVisualShapeIndex=hole_visual_shape_id,
    basePosition=hole_position
)

# Load the golf flag
flag_path = "C:/Users/hyper/Dropbox/PC/Downloads/golf-flag/source/yourMesh (3)/yourMesh (3).obj"
flag_visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName=flag_path,
    meshScale=[0.5, 0.5, 0.5],  # Scale down the flag for better visibility
    rgbaColor=[1, 1, 1, 1]  # White color for the flag
)

flag_position = [10, 10, np.min(heightmap) * height_multiplier + hole_height + 1]  # Position it next to the hole
flag_id = p.createMultiBody(
    baseMass=0,
    baseVisualShapeIndex=flag_visual_shape_id,
    basePosition=flag_position
)

# Setting gravity
p.setGravity(0, 0, -9.81)

# Set the camera to focus on the green area
p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0, 0, 0])

# Run the simulation
for i in range(2400):  # Run for 10 seconds (240 steps per second)
    if i % 240 == 0:
        ball_pos, _ = p.getBasePositionAndOrientation(ball_id)
        print(f"Step {i}: Ball position: {ball_pos}")
    p.stepSimulation()
    time.sleep(1/240)

# Final ball position
final_ball_pos, _ = p.getBasePositionAndOrientation(ball_id)
print(f"Final Ball position: {final_ball_pos}")

# Disconnect from PyBullet
p.disconnect()
