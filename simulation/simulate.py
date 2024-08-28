from __future__ import annotations

import random
import time
from typing import Any

import numpy as np
import pybullet as p
import pybullet_data
import scipy
from noise import pnoise2

from simulation.config import SimulationConfig
from simulation.config import TerrainConfig

Position = tuple[float, float, float]
Vertices = list[Position]
Indices = list[int]
Terrain = tuple[Vertices, Indices]


def generate_initial_positions(
    num_balls: int, config: TerrainConfig,
) -> list[Position]:
    buffer = 0.2 * config.width
    min_width, max_width = buffer, config.width - buffer

    def _generate() -> tuple[float]:
        return (
            random.uniform(min_width, max_width),
            random.uniform(min_width, max_width),
            2 * config.height,
        )

    return [_generate() for _ in range(num_balls)]


def generate_noisemap(config: TerrainConfig) -> np.ndarray:
    dimension = config.width * config.resolution
    heightmap = np.zeros((dimension, dimension))

    for i in range(dimension):
        for j in range(dimension):
            heightmap[i][j] = pnoise2(
                (i / config.resolution) / config.scale,
                (j / config.resolution) / config.scale,
                octaves=config.octaves,
                persistence=config.persistence,
                lacunarity=config.lacunarity,
            )

    # Smooth terrain with gaussian filter
    heightmap = scipy.ndimage.gaussian_filter(heightmap, config.filter_size)
    # Scale terrain heigh to be [0, config.height]
    old_min, old_max = heightmap.min(), heightmap.max()
    return (heightmap - old_min) * config.height / old_max


def generate_vertices(heightmap: np.ndarray, config: TerrainConfig) -> Terrain:
    vertices: Vertices = []
    indices: Indices = []

    width, height = heightmap.shape[0], heightmap.shape[1]

    for i in range(width):
        for j in range(height):
            # Each vertex is represented by (x, y, z), where:
            # x = column index (j), y = row index (i), z = height value
            ii, jj = i / config.resolution, j / config.resolution
            vertices.append((jj, ii, heightmap[i][j]))

    for i in range(width - 1):
        for j in range(height - 1):
            # Define two triangles for each grid square
            # Triangle 1
            i1 = i * height + j
            i2 = i1 + 1
            i3 = i1 + height
            # Triangle 2
            i4 = i3
            i5 = i2
            i6 = i3 + 1
            indices.extend([i1, i2, i3, i4, i5, i6])

    return vertices, indices


def _create_ball_body(
    position: Position,
    radius: float = 0.1,
    mass: float = 0.1,
    max_velocity: float = 1.0,
) -> Any:
    ball_visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=[1, 1, 1, 1],
        specularColor=[0.4, 0.4, 0],
    )

    ball_collision_shape_id = p.createCollisionShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius,
    )

    ball_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=ball_collision_shape_id,
        baseVisualShapeIndex=ball_visual_shape_id,
        basePosition=position,
    )

    p.changeDynamics(
        ball_id,
        -1,
        lateralFriction=0.2,
        rollingFriction=0.05,
        spinningFriction=0.05,
    )

    initial_velocity = [
        random.uniform(-max_velocity, max_velocity),
        random.uniform(-max_velocity, max_velocity),
        0,
    ]
    p.resetBaseVelocity(ball_id, linearVelocity=initial_velocity)

    return ball_id


def run_simulation(
    terrain: Terrain,
    positions: list[Position],
    sim_config: SimulationConfig,
    terrain_config: TerrainConfig,
    gui: bool = True,
) -> list[Position]:
    vertices, indices = terrain

    if gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setTimeStep(1 / sim_config.tick_rate)
    p.setGravity(0, 0, -9.81)

    collision_shape_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        meshScale=[1, 1, 1],
        vertices=vertices,
        indices=indices,
    )

    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        meshScale=[1, 1, 1],
        vertices=vertices,
        indices=indices,
        rgbaColor=[0, 1, 0, 1],
        specularColor=[0.4, 0.4, 0],
    )

    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape_id,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[0, 0, 0],
    )

    ball_ids = [
        _create_ball_body(
            position,
            radius=sim_config.ball_diameter / 2,
            mass=sim_config.ball_mass,
            max_velocity=0.1 * terrain_config.width,
        )
        for position in positions
    ]

    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(
            cameraDistance=terrain_config.width / 2,
            cameraYaw=0,
            cameraPitch=-55,
            cameraTargetPosition=[
                terrain_config.width / 2,
                0.3 * terrain_config.width,
                terrain_config.height / 2,
            ],
        )
        p.configureDebugVisualizer(shadowMapWorldSize=terrain_config.width)
        p.configureDebugVisualizer(shadowMapResolution=4096)
        p.configureDebugVisualizer(
            lightPosition=[
                10 * terrain_config.width,
                10 * terrain_config.width,
                10 * terrain_config.height,
            ],
        )

    for _ in range(sim_config.total_time * sim_config.tick_rate):
        p.stepSimulation()
        if gui:
            time.sleep(1 / sim_config.tick_rate)

    final_positions = [
        p.getBasePositionAndOrientation(ball_id)[0] for ball_id in ball_ids
    ]

    p.disconnect()

    return final_positions
