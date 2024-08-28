from __future__ import annotations

import matplotlib.pyplot as plt
import noise
import numpy as np
import scipy.ndimage
from scipy.stats import gaussian_kde


def generate_heatmap(positions, width, height, filepath):
    fig, axs = plt.subplots(
        1,
        2,
        figsize=(12, 6),
    )  # Two subplots for initial and final positions

    # Set up a grid of points for evaluation
    x = np.linspace(-width / 2, width / 2, 100)
    y = np.linspace(-height / 2, height / 2, 100)
    X, Y = np.meshgrid(x, y)

    for i, pos_list in enumerate(positions):
        if pos_list:
            flat_list = [
                (pos[0], pos[1])
                for pos in pos_list
                if isinstance(pos, (list, tuple)) and len(pos) >= 2
            ]
            if flat_list:
                x, y = zip(*flat_list)
                xy = np.vstack([x, y])
                kde = gaussian_kde(xy)
                Z = kde(np.vstack([X.ravel(), Y.ravel()])).reshape(X.shape)
                cs = axs[i].contourf(X, Y, Z, levels=25, cmap='jet')
                axs[i].set_title(
                    'Initial Positions' if i == 0 else 'Final Positions',
                )
            else:
                axs[i].text(
                    0.5,
                    0.5,
                    'No Data',
                    horizontalalignment='center',
                    verticalalignment='center',
                    transform=axs[i].transAxes,
                )
        else:
            axs[i].text(
                0.5,
                0.5,
                'No Data',
                horizontalalignment='center',
                verticalalignment='center',
                transform=axs[i].transAxes,
            )

        axs[i].set_xlim(-width / 2, width / 2)
        axs[i].set_ylim(-height / 2, height / 2)
        axs[i].set_xlabel('X position')
        axs[i].set_ylabel('Y position')
        axs[i].set_facecolor('green')

    fig.colorbar(cs, ax=axs, orientation='vertical')
    plt.savefig(filepath)


def generate_3d_heatmap(
    initial_positions,
    final_positions,
    width,
    height,
    filepath,
):
    # Parameters for terrain generation
    scale = 0.025  # Adjust this for different terrain scales
    octaves = 3  # More octaves means more detail in Perlin noise
    persistence = 0.2  # Increasing persistence ([0, 1]) increases roughness
    filter_size = (
        32  # Number of pixels per standard deviation for Gaussian filter
    )

    terrain = np.zeros((height, width))
    for y in range(height):
        for x in range(width):
            terrain[y, x] = noise.pnoise2(
                x * scale,
                y * scale,
                octaves=octaves,
                persistence=persistence,
            )

    terrain = scipy.ndimage.gaussian_filter(terrain, filter_size)

    x_coords, y_coords = np.meshgrid(
        np.linspace(-width / 2, width / 2, width),
        np.linspace(-height / 2, height / 2, height),
    )
    fig = plt.figure(figsize=(14, 7))
    axs = [fig.add_subplot(1, 2, i + 1, projection='3d') for i in range(2)]
    positions = [initial_positions, final_positions]
    titles = ['Initial Positions', 'Final Positions']

    for ax, pos_list, title in zip(axs, positions, titles):
        ax.set_facecolor('green')
        ax.plot_surface(
            x_coords,
            y_coords,
            terrain,
            cmap='Greens',
            alpha=0.6,
            edgecolor='none',
        )

        if pos_list:
            x, y, z = zip(*pos_list)
            xyz = np.vstack([x, y])
            kde = gaussian_kde(xyz)(xyz)
            kde_grid = np.zeros_like(terrain)
            for i, (xi, yi) in enumerate(zip(x, y)):
                xi_index = int((xi + width / 2) / width * width)
                yi_index = int((yi + height / 2) / height * height)
                kde_grid[yi_index, xi_index] = kde[i]

            kde_smoothed = scipy.ndimage.gaussian_filter(kde_grid, sigma=3)
            ax.plot_surface(
                x_coords,
                y_coords,
                terrain,
                facecolors=plt.cm.jet(kde_smoothed / kde_smoothed.max()),
                alpha=0.9,
            )

        ax.set_title(title)
        ax.set_xlim([-width / 2, width / 2])
        ax.set_ylim([-height / 2, height / 2])
        ax.set_zlim([terrain.min(), terrain.max()])

    plt.savefig(filepath)
