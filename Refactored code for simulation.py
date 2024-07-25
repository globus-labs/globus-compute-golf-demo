import pybullet as p
import pybullet_data
import time
import numpy as np
from noise import pnoise2
import random

def generate_noisemap(width, height, scale, octaves, persistence, lacunarity):
    heightmap = np.zeros((width, height))
    for i in range(width):
        for j in range(height):
            heightmap[i][j] = pnoise2(i / scale, j / scale, octaves=octaves, persistence=persistence, lacunarity=lacunarity)
    return heightmap

def generate_vertices(heightmap, width, height, height_multiplier):
    vertices = []
    indices = []
    for i in range(width):
        for j in range(height):
            vertices.append([i, j, heightmap[i][j] * height_multiplier])
    for i in range(width - 1):
        for j in range(height - 1):
            indices.append([i * height + j, (i + 1) * height + j, i * height + j + 1])
            indices.append([(i + 1) * height + j, (i + 1) * height + j + 1, i * height + j + 1])
    return vertices, indices

def run_simulation(vertices, indices, initial_ball_positions, gui=True):
    if gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")

    collision_shape_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        meshScale=[1, 1, 1],
        vertices=vertices,
        indices=[item for sublist in indices for item in sublist]  # Flatten indices list
    )

    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        meshScale=[1, 1, 1],
        vertices=vertices,
        indices=[item for sublist in indices for item in sublist],  # Flatten indices list
        rgbaColor=[0, 1, 0, 1],  # Opaque color for better visibility
        specularColor=[0.4, 0.4, 0]
    )
    # Getting maximum height of the green to position the green above the checkerboard plane
    max_height = np.max(np.array(vertices)[:, 2])

    # Creating the green
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[-width / 2, -height / 2, max_height]  # Center the green and lift it above the checkerboard plane
    )

    # Function to create and drop multiple golf balls
    def create_golf_ball(position, radius=0.1, mass=1):
        ball_visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=radius,
            rgbaColor=[1, 0, 0, 1],  # Opaque red color for better visibility
            specularColor=[0.4, 0.4, 0]
        )

        ball_collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_SPHERE,
            radius=radius
        )

        ball_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=ball_collision_shape_id,
            baseVisualShapeIndex=ball_visual_shape_id,
            basePosition=position
        )
        
        # Setting the friction and damping properties for realistic rolling behavior
        p.changeDynamics(ball_id, -1, lateralFriction=0.3, rollingFriction=0.05, spinningFriction=0.05, linearDamping=0.05, angularDamping=0.05)
        
        # Applying an initial velocity to the ball
        initial_velocity = [random.uniform(-2, 2), random.uniform(-2, 2), 0]
        p.resetBaseVelocity(ball_id, linearVelocity=initial_velocity)
        
        return ball_id

    # Create and drop the balls
    ball_ids = []
    for position in initial_ball_positions:
        ball_id = create_golf_ball(position)
        ball_ids.append(ball_id)

    # Setting gravity
    p.setGravity(0, 0, -9.81)

    # Setting the camera to focus on the green area if GUI is enabled
    if gui:
        p.resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0, 0, 0])

    # Run the simulation
    for _ in range(2400):  # Run for 10 seconds (240 steps per second)
        p.stepSimulation()
        if gui:
            time.sleep(1/240)

    # Get final ball positions
    final_ball_positions = [p.getBasePositionAndOrientation(ball_id)[0] for ball_id in ball_ids]

    # Disconnect from PyBullet
    p.disconnect()

    return final_ball_positions

# example
width = 100
height = 100
scale = 10.0
octaves = 2
persistence = 0.5
lacunarity = 2.0
height_multiplier = 2.0

heightmap = generate_noisemap(width, height, scale, octaves, persistence, lacunarity)
vertices, indices = generate_vertices(heightmap, width, height, height_multiplier)

# Initial ball positions around the center of the green
initial_ball_positions = [[random.uniform(-1, 1), random.uniform(-1, 1), np.max(heightmap) * height_multiplier + 5] for _ in range(10)]

# Run the simulation with GUI
final_positions = run_simulation(vertices, indices, initial_ball_positions, gui=True)
print("Final ball positions with GUI:", final_positions)

# Run the simulation without GUI (headless)
final_positions_headless = run_simulation(vertices, indices, initial_ball_positions, gui=False)
print("Final ball positions headless:", final_positions_headless)

