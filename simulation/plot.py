from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.stats import gaussian_kde

from simulation.config import TerrainConfig
from simulation.simulate import Position


def create_contour_plot(
    initial_positions: list[Position],
    final_positions: list[Position],
    heightmap: np.ndarray,
    config: TerrainConfig,
    filepath: str,
) -> None:
    fig, axs = plt.subplots(1, 2, sharey=True, figsize=(8, 4))

    x = np.linspace(0, config.width, num=config.width * config.resolution)
    y = np.linspace(0, config.width, num=config.width * config.resolution)
    for ax, positions in zip(axs, (initial_positions, final_positions)):
        handle = ax.contour(x, y, heightmap, levels=10)
        plt.clabel(handle, inline=True)
        px, py = (
            # Clamp balls that may have rolled off the surface to the
            # edge of the plot.
            [max(0, min(p[0], config.width)) for p in positions],
            [max(0, min(p[1], config.width)) for p in positions],
        )
        ax.scatter(px, py, s=16, c='#FFFFFF', zorder=100)

    for i, (ax, title) in enumerate(zip(axs, ('Initial', 'Final'))):
        ax.set_title(title)
        ax.set_xlim(0, config.width)
        ax.set_ylim(0, config.width)
        ax.set_xlabel('X Position (m)')
        if i == 0:
            ax.set_ylabel('Y Position (m)')
        ax.set_facecolor('#58a177')

    fig.tight_layout(w_pad=2)
    fig.savefig(filepath, pad_inches=0.05, dpi=300)


def create_terrain_plot(
    initial_positions: list[Position],
    final_positions: list[Position],
    heightmap: np.ndarray,
    config: TerrainConfig,
    filepath: str,
) -> None:
    fig = plt.figure(figsize=(8, 4))
    axs = [
        fig.add_subplot(1, 2, 1, projection='3d'),
        fig.add_subplot(1, 2, 2, projection='3d'),
    ]
    plt.subplots_adjust(wspace=0.5, hspace=0.5)

    x = np.arange(0, config.width, 1 / config.resolution)
    y = np.arange(0, config.width, 1 / config.resolution)
    x, y = np.meshgrid(x, y)

    for ax, positions in zip(axs, (initial_positions, final_positions)):
        px, py, pz = zip(*positions)
        xy = np.vstack([px, py])
        kde = gaussian_kde(xy)(xy)
        kde_grid = np.zeros_like(heightmap)

        for i, (xi, yi) in enumerate(zip(px, py)):
            # Clamp positions to be in [0, config.width]. If a ball rolled
            # off the edge, it's position would be outside the mesh map.
            max_index = (config.width * config.resolution) - 1
            xi_scaled = int(xi * config.resolution)
            yi_scaled = int(yi * config.resolution)
            xi_scaled = max(0, min(xi_scaled, max_index))
            yi_scaled = max(0, min(yi_scaled, max_index))
            kde_grid[yi_scaled, xi_scaled] = kde[i]

        kde_smoothed = gaussian_filter(kde_grid, sigma=1)
        ax.plot_surface(
            x,
            y,
            heightmap,
            facecolors=plt.cm.jet(kde_smoothed / kde_smoothed.max()),
            alpha=0.9,
        )

    for ax, title in zip(axs, ('Initial', 'Final')):
        ax.set_title(title, pad=-20)
        ax.set_xlim(0, config.width)
        ax.set_ylim(0, config.width)
        ax.set_zlim(0, 2 * heightmap.max())
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.set_zlabel('Z Position (m)')

    fig.tight_layout()
    fig.subplots_adjust(wspace=0.15, left=0, right=0.92, bottom=0.05, top=0.98)
    fig.savefig(filepath, dpi=300)
