import pybullet as p
import pybullet_data
import time
import numpy as np
from noise import pnoise2
import random
import math

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

def run_simulation(vertices, indices, initial_ball_positions, width, height, gui=True):
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
        indices=[item for sublist in indices for item in sublist]  
    )

    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        meshScale=[1, 1, 1],
        vertices=vertices,
        indices=[item for sublist in indices for item in sublist], 
        rgbaColor=[0, 1, 0, 1], 
        specularColor=[0.4, 0.4, 0]
    )

    max_height = np.max(np.array(vertices)[:, 2])

    # Creating the green
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[-width / 2, -height / 2, max_height]  # Center the green and lift it above the checkerboard plane
    )

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
        
        # Set the friction and damping properties for realistic rolling behavior
        p.changeDynamics(ball_id, -1, lateralFriction=0.3, rollingFriction=0.05, spinningFriction=0.05, linearDamping=0.05, angularDamping=0.05)
        
        # Apply an initial velocity to the ball for faster rolling
        initial_velocity = [random.uniform(-2, 2), random.uniform(-2, 2), 0]
        p.resetBaseVelocity(ball_id, linearVelocity=initial_velocity)
        
        return ball_id

    ball_ids = []
    for position in initial_ball_positions:
        ball_id = create_golf_ball(position)
        ball_ids.append(ball_id)

    # Setting gravity
    p.setGravity(0, 0, -9.81)

    # Set the camera to focus on the green area if GUI is enabled
    if gui:
        p.resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0, 0, 0])
        p.configureDebugVisualizer(shadowMapWorldSize=10)
        p.configureDebugVisualizer(shadowMapResolution=4096)
        p.configureDebugVisualizer(lightPosition=[0, 20, 10])

    # Run the simulation
    for i in range(2400):  # Run for 10 seconds (240 steps per second)
        p.stepSimulation()
        if gui:
            if i % 240 == 0:  # Adjust light position every second
                angle = (i / 240) * (2 * math.pi)
                #p.configureDebugVisualizer(lightPosition=[5 * math.sin(angle), 5 * math.cos(angle), 5])
            time.sleep(1 / 240)

    # Get final ball positions
    final_ball_positions = [p.getBasePositionAndOrientation(ball_id)[0] for ball_id in ball_ids]

    # Disconnect from PyBullet
    p.disconnect()

    return final_ball_positions

def generate_heatmap(final_positions, width, height):
    import matplotlib.pyplot as plt
    from scipy.stats import gaussian_kde

    x, y = zip(*[(pos[0], pos[1]) for pos in final_positions])
    xy = np.vstack([x, y])
    z = gaussian_kde(xy)(xy)

    fig, ax = plt.subplots()
    ax.set_facecolor('green')
    
    sc = ax.scatter(x, y, c=z, s=100, cmap='jet', label=['Ball'+str(i+1) for i in range(len(final_positions))])
    
    # Create a legend
    handles, _ = sc.legend_elements(prop="colors", alpha=0.6)
    labels = [f'Ball{i+1}' for i in range(len(final_positions))]
    legend = ax.legend(handles, labels, loc="upper right", title="Balls")

    plt.colorbar(sc)
    plt.title("Heatmap of Ball Positions")
    plt.xlabel("X position")
    plt.ylabel("Y position")
    plt.xlim(-width/2, width/2)
    plt.ylim(-height/2, height/2)

    # Ensure the plot remains open
    plt.ioff()
    plt.show()





