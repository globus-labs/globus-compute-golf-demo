from __future__ import annotations

import logging
import math
import random
import sys
import time

import numpy as np
import pybullet as p
import pybullet_data
import scipy
from noise import pnoise2
from numpy.typing import NDArray

from simulation.config import SimulationConfig
from simulation.config import TerrainConfig

logger = logging.getLogger(__name__)

Position = tuple[float, float, float]
Vertices = list[Position]
Indices = list[int]
TerrainPart = tuple[Vertices, Indices]
# The two floats here represent the x, y positions where the terrain
# part starts at.
Terrain = list[tuple[float, float, TerrainPart]]

# PyBullet has a limit of 131,072 vertices for a mesh map on Windows/Linux
# but only 8,192 on MacOS due to the smaller shared memory default on MacOS.
#   https://github.com/bulletphysics/bullet3/issues/1965
# So we need to split our terrain into smaller chunks. This parameter
# controls the maximum number of points in each chunk.
# These numbers are chosen to be half the max vertex count in pybullet.
#  https://github.com/bulletphysics/bullet3/blob/e9c461b0ace140d5c73972760781d94b7b5eee53/examples/SharedMemory/SharedMemoryPublic.h#L1128-L1134
if sys.platform == 'darwin':
    MAX_VERTICES_PER_MESH = 4192
else:
    MAX_VERTICES_PER_MESH = 65536


def generate_initial_positions(
    num_balls: int,
    config: TerrainConfig,
    seed: int | None = None,
) -> list[Position]:
    buffer = 0.2 * config.width
    min_width, max_width = buffer, config.width - buffer

    random.seed(seed)

    def _generate() -> Position:
        return (
            random.uniform(min_width, max_width),
            random.uniform(min_width, max_width),
            2 * config.height,
        )

    return [_generate() for _ in range(num_balls)]


def generate_noisemap(config: TerrainConfig) -> NDArray[np.float64]:
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
    # Scale terrain height to be [0, config.height]
    old_min, old_max = heightmap.min(), heightmap.max()
    return (heightmap - old_min) * config.height / old_max


def _generate_vertices(
    heightmap: NDArray[np.float64],
    resolution: int,
) -> TerrainPart:
    vertices: Vertices = []
    indices: Indices = []

    width, height = heightmap.shape[0], heightmap.shape[1]

    for i in range(width):
        for j in range(height):
            # Each vertex is represented by (x, y, z), where:
            # x = column index (j), y = row index (i), z = height value
            ii, jj = i / resolution, j / resolution
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


def generate_vertices(
    heightmap: NDArray[np.float64],
    config: TerrainConfig,
) -> Terrain:
    width, height = heightmap.shape[0], heightmap.shape[1]
    vertices = width * height
    parts = math.ceil(vertices / MAX_VERTICES_PER_MESH)
    max_width_per_part = math.ceil(width / parts)

    terrain: Terrain = []
    for x1 in range(0, width, max_width_per_part):
        # Add one to right index of slice so that the terrain parts
        # overlap each other slightly.
        x2 = min(width, x1 + max_width_per_part + 1)
        # For now, we only split on axis 0 (referred to as x-axis).
        y = 0 / config.resolution
        heightmap_part = heightmap[x1:x2, :]
        terrain_part = _generate_vertices(heightmap_part, config.resolution)
        terrain.append((x1 / config.resolution, y, terrain_part))

    return terrain


def _create_terrain_body(terrain: Terrain) -> list[int]:
    terrain_ids: list[int] = []

    for part in terrain:
        x, y, (vertices, indices) = part

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
            rgbaColor=[88 / 255, 161 / 255, 119 / 255, 1],
            specularColor=[0.4, 0.4, 0],
        )

        terrain_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=[y, x, 0],
        )

        terrain_ids.append(terrain_id)

    return terrain_ids


def _create_ball_body(
    position: Position,
    radius: float = 0.1,
    mass: float = 0.1,
    max_velocity: float = 1.0,
) -> int:
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
    if gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setTimeStep(1 / sim_config.tick_rate)
    p.setGravity(0, 0, -9.81)

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

    _create_terrain_body(terrain)

    ball_ids = [
        _create_ball_body(
            position,
            radius=sim_config.ball_diameter / 2,
            mass=sim_config.ball_mass,
            max_velocity=0.1 * terrain_config.width,
        )
        for position in positions
    ]

    logger.debug(
        f'Simulating for {sim_config.total_time} '
        f'({sim_config.tick_rate} steps per seconds)',
    )

    for _ in range(sim_config.total_time * sim_config.tick_rate):
        p.stepSimulation()
        if gui:
            time.sleep(1 / sim_config.tick_rate)

    logger.debug('Simulation completed')

    final_positions = [
        p.getBasePositionAndOrientation(ball_id)[0] for ball_id in ball_ids
    ]

    p.disconnect()

    return final_positions
