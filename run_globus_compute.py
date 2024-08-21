import argparse
from globus_compute_sdk import Executor

def simulation_task(ball_position, width, height, scale, octaves, persistence, lacunarity, height_multiplier):
    from simulation import generate_noisemap, generate_vertices, run_simulation  
    heightmap = generate_noisemap(width, height, scale, octaves, persistence, lacunarity)
    vertices, indices = generate_vertices(heightmap, width, height, height_multiplier)
    final_position = run_simulation(vertices, indices, [ball_position], width, height, gui=False)
    return ball_position, final_position  

def main():
    import random
    parser = argparse.ArgumentParser(description="Run golf ball simulation on Globus Compute.")
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

    args = parser.parse_args()

    initial_ball_positions = [[random.uniform(-1, 1), random.uniform(-1, 1), 0] for _ in range(args.num_balls)]

    if args.globus_compute:
        with Executor(endpoint_id=args.endpoint_id) as fxe:
            futures = [fxe.submit(simulation_task, pos, args.width, args.height, args.scale, args.octaves, args.persistence, args.lacunarity, args.height_multiplier) for pos in initial_ball_positions]
            results = [future.result() for future in futures]
            for init_pos, final_pos in results:
                print("Initial position: {}, Final position: {}".format(init_pos, final_pos))
    else:
        for pos in initial_ball_positions:
            init_pos, final_pos = simulation_task(pos, args.width, args.height, args.scale, args.octaves, args.persistence, args.lacunarity, args.height_multiplier)
            print("Initial position: {}, Final position: {}".format(init_pos, final_pos))

if __name__ == "__main__":
    main()



