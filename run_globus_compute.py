from __future__ import annotations

import argparse
import logging
import sys
from concurrent.futures import ProcessPoolExecutor
from typing import Sequence

from globus_compute_sdk import Executor

from simulation.config import SimulationConfig
from simulation.config import TerrainConfig
from simulation.plot import create_contour_plot
from simulation.plot import create_terrain_plot
from simulation.simulate import generate_initial_positions
from simulation.simulate import generate_noisemap
from simulation.simulate import generate_vertices
from simulation.simulate import run_simulation

logger = logging.getLogger('demo')


def main(argv: Sequence[str] | None = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description='Simulate a golf ball',
    )
    parser.add_argument(
        '--num-balls',
        type=int,
        required=True,
        help='number of balls to simulate',
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        '--endpoint',
        type=str,
        help='globus compute endpoint UUID',
    )
    group.add_argument(
        '--process-pool',
        type=int,
        help='number of local process to execute simulation across',
    )
    group.add_argument(
        '--contour-plot',
        default='images/contour.png',
        help='filepath for saving contour plot',
    )
    group.add_argument(
        '--terrain-plot',
        default='images/terrain.png',
        help='filepath for saving 3D terrain plot',
    )
    SimulationConfig.add_argument_group(parser)
    TerrainConfig.add_argument_group(parser)
    args = parser.parse_args(argv)

    logging.basicConfig(
        format='[%(levelname)s - %(asctime)s] (%(name)s) > %(message)s',
        level=logging.INFO,
        stream=sys.stdout,
    )

    sim_config = SimulationConfig.from_args(args)
    terrain_config = TerrainConfig.from_args(args)

    terrain_heightmap = generate_noisemap(terrain_config)
    terrain_mesh = generate_vertices(terrain_heightmap, terrain_config)

    initial_positions = generate_initial_positions(
        args.num_balls,
        terrain_config,
        seed=sim_config.seed,
    )
    logger.info(f'Generated {len(initial_positions)} initial position(s)')

    if args.endpoint:
        executor = Executor(endpoint_id=args.endpoint)
        logger.info(
            f'Initialized Globus Compute Executor with UUID {args.endpoint}',
        )
    else:
        executor = ProcessPoolExecutor(args.process_pool)
        logger.info(
            f'Initialized process pool with {args.process_pool} workers',
        )

    logger.info('Submitting simulations')
    with executor:
        futures = [
            executor.submit(
                run_simulation,
                terrain_mesh,
                [position],
                sim_config=sim_config,
                terrain_config=terrain_config,
                gui=False,
            )
            for position in initial_positions
        ]
        logger.info('Simulations submitted')

        results = [future.result() for future in futures]
        final_positions = [pos for result in results for pos in result]

    logger.info(f'Received {len(final_positions)} final position(s)')

    create_contour_plot(
        initial_positions,
        final_positions,
        terrain_heightmap,
        terrain_config,
        args.contour_plot,
    )
    logger.info(f'Saved contour map to {args.contour_plot}')

    create_terrain_plot(
        initial_positions,
        final_positions,
        terrain_heightmap,
        terrain_config,
        args.terrain_plot,
    )
    logger.info(f'Saved terrain map to {args.terrain_plot}')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
