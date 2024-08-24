import argparse
from globus_compute_sdk import Executor

def main():
    from simulation import generate_noisemap, generate_vertices, run_simulation, generate_heatmap, generate_3d_heatmap
    import random
    import numpy as np
    parser = argparse.ArgumentParser(description="Run golf ball simulation and generate heatmaps on Globus Compute.")
    parser.add_argument("--width", type=int, default=100, help="Width of the green")
    parser.add_argument("--height", type=int, default=100, help="Height of the green")
    parser.add_argument("--scale", type=float, default=10.0, help="Scale of the noise map")
    parser.add_argument("--octaves", type=int, default=2, help="Number of octaves for noise generation")
    parser.add_argument("--persistence", type=float, default=0.5, help="Persistence of the noise")
    parser.add_argument("--lacunarity", type=float, default=2.0, help="Lacunarity of the noise")
    parser.add_argument("--height_multiplier", type=float, default=2.0, help="Height multiplier for the terrain")
    parser.add_argument("--num_balls", type=int, default=10, help="Number of balls to simulate")
    parser.add_argument("--gui", action="store_true", help="Run with GUI (default: headless)")
    parser.add_argument("--globus_compute", action="store_true", help="Submit the task to Globus Compute")
    parser.add_argument("--endpoint_id", type=str, default=None, help="Globus Compute endpoint ID")
    parser.add_argument("--output_path_2d", type=str, default="heatmap_output_2d.png", help="File path for the output 2D heatmap")
    parser.add_argument("--output_path_3d", type=str, default="heatmap_output_3d.png", help="File path for the output 3D heatmap")

    args = parser.parse_args()
    
    heightmap = generate_noisemap(args.width, args.height, args.scale, args.octaves, args.persistence, args.lacunarity)
    vertices, indices = generate_vertices(heightmap, args.width, args.height, args.height_multiplier)

    start_z = np.max(heightmap) * args.height_multiplier + 5

    initial_ball_positions = [
        [random.uniform(-args.width / 4, args.width / 4), random.uniform(-args.height / 4, args.height / 4), start_z]
        for _ in range(args.num_balls)]

    if args.globus_compute:
        with Executor(endpoint_id=args.endpoint_id) as fxe:
            futures = [fxe.submit(run_simulation, vertices, indices, [pos], args.width, args.height, gui=False) for pos in initial_ball_positions]
            results = [future.result() for future in futures]
            final_positions = [x for result in results for x in result]  # Flatten the list of lists
    else:
        results = [run_simulation(vertices, indices, [pos], args.width, args.height, gui=False) for pos in initial_ball_positions]
        final_positions = [x for result in results for x in result]  

    for init_pos, final_pos in zip(initial_ball_positions, final_positions):
        print(f"Initial position: {init_pos}, Final position: {final_pos}")

    generate_heatmap([initial_ball_positions, final_positions], args.width, args.height, args.output_path_2d)
    generate_3d_heatmap(initial_ball_positions, final_positions, args.width, args.height, args.output_path_3d)

if __name__ == "__main__":
    main()







