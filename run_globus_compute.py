import argparse
from globus_compute_sdk import Executor

def simulation_task(width, height, scale, octaves, persistence, lacunarity, height_multiplier, num_balls, gui):
    import numpy as np
    import random
    from simulation import generate_noisemap, generate_vertices, run_simulation, generate_heatmap
    heightmap = generate_noisemap(width, height, scale, octaves, persistence, lacunarity)
    vertices, indices = generate_vertices(heightmap, width, height, height_multiplier)

    initial_ball_positions = [[random.uniform(-1, 1), random.uniform(-1, 1), np.max(heightmap) * height_multiplier + 5] for _ in range(num_balls)]

    final_positions = run_simulation(vertices, indices, initial_ball_positions, width, height, gui=gui)
    print("Final ball positions:", final_positions)
    
    if not gui:
        generate_heatmap(final_positions, width, height)
    return final_positions

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
    parser.add_argument("--globus_compute", action="store_true", help="Submit the task to Globus Compute")
    parser.add_argument("--endpoint_id", type=str, default='cd492892-7c6a-4638-b1d3-bb6fae059455', help="Globus Compute endpoint ID")

    args = parser.parse_args()
    input_params = (args.width, args.height, args.scale, args.octaves, args.persistence, args.lacunarity, args.height_multiplier, args.num_balls, args.gui)

    if args.globus_compute:
        with Executor(endpoint_id=args.endpoint_id) as fxe:
            future = fxe.submit(simulation_task, *input_params)
            result = future.result()
            print("Result from Globus Compute:", result)
    else:
        simulation_task(*input_params)

if __name__ == "__main__":
    main()
