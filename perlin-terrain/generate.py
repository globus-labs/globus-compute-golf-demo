import matplotlib.pyplot as plt
import noise
import numpy as np
import scipy.ndimage

# Parameters for terrain generation
width = 256
height = 256
scale = 0.025  # Adjust this for different terrain scales
octaves = 3  # More octaves means more detail in perlin noise
persistence = 0.2  # Increasing persistence ([0, 1]) increases roughness
filter_size = 32  # Number of pixels per standard deviation for gaussian filter

# Initialize an empty 3D array to store terrain data
terrain = np.zeros((height, width))

# Generate terrain using Perlin noise
for y in range(height):
    for x in range(width):
        value = noise.pnoise2(
            x * scale,
            y * scale,
            octaves=octaves,
            persistence=persistence,
        )
        terrain[y][x] = value

# Smooth out the map
terrain = scipy.ndimage.gaussian_filter(terrain, filter_size)

# Create a 3D visualization of the terrain
x_coords, y_coords = np.meshgrid(np.arange(width), np.arange(height))
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_zlim3d(-1, 1)
terrain_plot = ax.scatter(
    x_coords, y_coords, terrain, c=terrain, cmap="terrain", marker=".",
)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

plt.savefig("terrain.png", dpi=600)
