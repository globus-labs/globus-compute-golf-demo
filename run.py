import argparse
import numpy as np
import random
from simulation import generate_noisemap, generate_vertices, run_simulation, generate_heatmap, generate_3d_heatmap

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
    parser.add_argument("--output_path_2d", type=str, default="heatmap_output_2d.png", help="File path for the output 2D heatmap")
    parser.add_argument("--output_path_3d", type=str, default="heatmap_output_3d.png", help="File path for the output 3D heatmap")

    args = parser.parse_args()

    heightmap = generate_noisemap(args.width, args.height, args.scale, args.octaves, args.persistence, args.lacunarity)
    vertices, indices = generate_vertices(heightmap, args.width, args.height, args.height_multiplier)

    # Calculate the proper starting z-coordinate based on the terrain and desired offset
    start_z = np.max(heightmap) * args.height_multiplier + 5

    initial_ball_positions = [
        [random.uniform(-args.width / 4, args.width / 4), random.uniform(-args.height / 4, args.height / 4), start_z]
        for _ in range(args.num_balls)
]
    
    print("Initial ball positions:", initial_ball_positions)

    final_positions = run_simulation(vertices, indices, initial_ball_positions, args.width, args.height, gui=args.gui)
    print("Final ball positions:", final_positions)
    generate_heatmap([initial_ball_positions, final_positions], args.width, args.height, args.output_path_2d)
    generate_3d_heatmap(initial_ball_positions, final_positions, args.width, args.height, args.output_path_3d)

if __name__ == "__main__":
    main()



