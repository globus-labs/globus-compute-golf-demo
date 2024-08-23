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

    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[-width / 2, -height / 2, max_height] 
    )

    def create_golf_ball(position, radius=0.1, mass=1):
        ball_visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=radius,
            rgbaColor=[1, 0, 0, 1], 
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
        
        p.changeDynamics(ball_id, -1, lateralFriction=0.3, rollingFriction=0.05, spinningFriction=0.05, linearDamping=0.05, angularDamping=0.05)
        
        initial_velocity = [random.uniform(-2, 2), random.uniform(-2, 2), 0]
        p.resetBaseVelocity(ball_id, linearVelocity=initial_velocity)
        
        return ball_id

    ball_ids = []
    for position in initial_ball_positions:
        ball_id = create_golf_ball(position)
        ball_ids.append(ball_id)

    p.setGravity(0, 0, -9.81)

    if gui:
        p.resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0, 0, 0])
        p.configureDebugVisualizer(shadowMapWorldSize=10)
        p.configureDebugVisualizer(shadowMapResolution=4096)
        p.configureDebugVisualizer(lightPosition=[0, 20, 10])

    for i in range(2400): 
        p.stepSimulation()
        if gui:
            if i % 240 == 0: 
                angle = (i / 240) * (2 * math.pi)
            time.sleep(1 / 240)

    final_ball_positions = [p.getBasePositionAndOrientation(ball_id)[0] for ball_id in ball_ids]
    
    p.disconnect()

    return final_ball_positions

def generate_heatmap(positions, width, height, filepath):
    import matplotlib.pyplot as plt
    from scipy.stats import gaussian_kde

    fig, axs = plt.subplots(1, 2, figsize=(12, 6))  

    for i, pos_list in enumerate(positions):
        if pos_list:  
            flat_list = []
            for pos in pos_list:
                if isinstance(pos, tuple) or isinstance(pos, list):
                    if len(pos) >= 2:
                        flat_list.append(pos)
                elif isinstance(pos, tuple) or isinstance(pos, list) and len(pos[0]) >= 2:
                    flat_list.append(pos[0])

            if flat_list:  
                x, y = zip(*[(p[0], p[1]) for p in flat_list])
                xy = np.vstack([x, y])
                z = gaussian_kde(xy)(xy)
                sc = axs[i].scatter(x, y, c=z, s=100, cmap='jet')
                axs[i].set_title('Initial Positions' if i == 0 else 'Final Positions')
                axs[i].set_xlim(-width/2, width/2)
                axs[i].set_ylim(-height/2, height/2)
            else:
                axs[i].text(0.5, 0.5, 'No Data', horizontalalignment='center', verticalalignment='center', transform=axs[i].transAxes)
        else:
            axs[i].text(0.5, 0.5, 'No Data', horizontalalignment='center', verticalalignment='center', transform=axs[i].transAxes)

        axs[i].set_xlabel("X position")
        axs[i].set_ylabel("Y position")
        axs[i].set_facecolor('green')

    plt.colorbar(sc, ax=axs, location='right')
    plt.savefig(filepath)

def generate_3d_heatmap(initial_positions, final_positions, width, height, filepath):
    import numpy as np
    import matplotlib.pyplot as plt
    from scipy.stats import gaussian_kde
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure(figsize=(14, 7))
    axs = [fig.add_subplot(1, 2, i+1, projection='3d') for i in range(2)]
    positions = [initial_positions, final_positions]
    titles = ['Initial Positions', 'Final Positions']

    for ax, pos_list, title in zip(axs, positions, titles):
        ax.set_facecolor('green')
        if pos_list:
            processed_positions = [(pos[0] + np.random.normal(0, 0.01), pos[1] + np.random.normal(0, 0.01), pos[2] + np.random.normal(0, 0.01)) 
                                   for pos in pos_list if isinstance(pos, (list, tuple)) and len(pos) == 3]
            if processed_positions:
                x, y, z = zip(*processed_positions)
                xyz = np.vstack([x, y, z])
                kde = gaussian_kde(xyz)(xyz)
                sc = ax.scatter(x, y, z, c=kde, s=100, cmap='jet')
                plt.colorbar(sc, ax=ax)

        ax.set_title(title)
        ax.set_xlim([-width/2, width/2])
        ax.set_ylim([-height/2, height/2])
        ax.set_zlim([0, max(z) * 1.2 if processed_positions else 1])
        ax.set_xlabel("X position")
        ax.set_ylabel("Y position")
        ax.set_zlabel("Z position")

    plt.savefig(filepath)





