import argparse
import numpy as np
import random
from simulation import generate_noisemap, generate_vertices, run_simulation, generate_heatmap

def main():
    parser = argparse.ArgumentParser(description="Run golf ball simulation.")
    parser.add_argument("--width", type=int, default=100, help="Width of the green")
    parser.add_argument("--height", type=int, default=100, help="Height of the green")
    parser.add_argument("--scale", type=float, default=10.0, help="Scale of the noise map")
    parser.add_argument("--octaves", type=int, default=2, help="Number of octaves for noise generation")
    parser.add_argument("--persistence", type=float, default=0.5, help="Persistence of the noise")
    parser.add_argument("--lacunarity", type=float, default=2.0, help="Lacunarity of the noise")
    parser.add_argument("--height_multiplier", type=float, default=2.0, help="Height multiplier for the terrain")
    parser.add_argument("--num_balls", type=int, default=10, help="Number of balls to simulate")
    parser.add_argument("--gui", action="store_true", help="Run with GUI (default: headless)")

    args = parser.parse_args()

    # Generate noise map and vertices
    heightmap = generate_noisemap(args.width, args.height, args.scale, args.octaves, args.persistence, args.lacunarity)
    vertices, indices = generate_vertices(heightmap, args.width, args.height, args.height_multiplier)

    # Initial ball positions around the center of the green
    initial_ball_positions = [[random.uniform(-1, 1), random.uniform(-1, 1), np.max(heightmap) * args.height_multiplier + 5] for _ in range(args.num_balls)]

    # Run the simulation
    final_positions = run_simulation(vertices, indices, initial_ball_positions, args.width, args.height, gui=args.gui)
    print("Final ball positions:", final_positions)

    # Generate and display heatmap
    generate_heatmap(final_positions, args.width, args.height)

if __name__ == "__main__":
    main()



