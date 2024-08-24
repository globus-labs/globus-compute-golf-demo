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
    import numpy as np
    import matplotlib.pyplot as plt
    from scipy.stats import gaussian_kde

    fig, axs = plt.subplots(1, 2, figsize=(12, 6))  # Two subplots for initial and final positions

    # Set up a grid of points for evaluation
    x = np.linspace(-width/2, width/2, 100)
    y = np.linspace(-height/2, height/2, 100)
    X, Y = np.meshgrid(x, y)

    for i, pos_list in enumerate(positions):
        if pos_list:  
            flat_list = [(pos[0], pos[1]) for pos in pos_list if isinstance(pos, (list, tuple)) and len(pos) >= 2]
            if flat_list:
                x, y = zip(*flat_list)
                xy = np.vstack([x, y])
                kde = gaussian_kde(xy)
                Z = kde(np.vstack([X.ravel(), Y.ravel()])).reshape(X.shape)
                cs = axs[i].contourf(X, Y, Z, levels=25, cmap='jet')
                axs[i].set_title('Initial Positions' if i == 0 else 'Final Positions')
            else:
                axs[i].text(0.5, 0.5, 'No Data', horizontalalignment='center', verticalalignment='center', transform=axs[i].transAxes)
        else:
            axs[i].text(0.5, 0.5, 'No Data', horizontalalignment='center', verticalalignment='center', transform=axs[i].transAxes)

        axs[i].set_xlim(-width/2, width/2)
        axs[i].set_ylim(-height/2, height/2)
        axs[i].set_xlabel("X position")
        axs[i].set_ylabel("Y position")
        axs[i].set_facecolor('green')

    fig.colorbar(cs, ax=axs, orientation='vertical')
    plt.savefig(filepath)


def generate_3d_heatmap(initial_positions, final_positions, width, height, filepath):
    import numpy as np
    import matplotlib.pyplot as plt
    import noise
    import scipy.ndimage
    from scipy.stats import gaussian_kde
    from mpl_toolkits.mplot3d import Axes3D

    # Parameters for terrain generation
    scale = 0.025  # Adjust this for different terrain scales
    octaves = 3  # More octaves means more detail in Perlin noise
    persistence = 0.2  # Increasing persistence ([0, 1]) increases roughness
    filter_size = 32  # Number of pixels per standard deviation for Gaussian filter

    terrain = np.zeros((height, width))
    for y in range(height):
        for x in range(width):
            terrain[y, x] = noise.pnoise2(x * scale, y * scale, octaves=octaves, persistence=persistence)

    terrain = scipy.ndimage.gaussian_filter(terrain, filter_size)

    x_coords, y_coords = np.meshgrid(np.linspace(-width/2, width/2, width), np.linspace(-height/2, height/2, height))
    fig = plt.figure(figsize=(14, 7))
    axs = [fig.add_subplot(1, 2, i+1, projection='3d') for i in range(2)]
    positions = [initial_positions, final_positions]
    titles = ['Initial Positions', 'Final Positions']

    for ax, pos_list, title in zip(axs, positions, titles):
        ax.set_facecolor('green')
        ax.plot_surface(x_coords, y_coords, terrain, cmap='Greens', alpha=0.6, edgecolor='none')

        if pos_list:
            x, y, z = zip(*pos_list)
            xyz = np.vstack([x, y])
            kde = gaussian_kde(xyz)(xyz)
            kde_grid = np.zeros_like(terrain)
            for i, (xi, yi) in enumerate(zip(x, y)):
                xi_index = int((xi + width/2) / width * width)
                yi_index = int((yi + height/2) / height * height)
                kde_grid[yi_index, xi_index] = kde[i]

            kde_smoothed = scipy.ndimage.gaussian_filter(kde_grid, sigma=3) 
            ax.plot_surface(x_coords, y_coords, terrain, facecolors=plt.cm.jet(kde_smoothed/kde_smoothed.max()), alpha=0.9)

        ax.set_title(title)
        ax.set_xlim([-width/2, width/2])
        ax.set_ylim([-height/2, height/2])
        ax.set_zlim([terrain.min(), terrain.max()])

    plt.savefig(filepath)





