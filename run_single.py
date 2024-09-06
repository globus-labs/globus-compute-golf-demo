from __future__ import annotations

import argparse
import logging
import sys
from typing import Sequence

from simulation.config import SimulationConfig
from simulation.config import TerrainConfig
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
        default=1,
        type=int,
        help='number of balls to simulate',
    )
    parser.add_argument(
        '--gui',
        default=True,
        action=argparse.BooleanOptionalAction,
        help='run with the gui',
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

    terrain_heightmap = generate_noisemap(terrain_config, seed=sim_config.seed)
    terrain_mesh = generate_vertices(terrain_heightmap, terrain_config)

    initial_positions = generate_initial_positions(
        args.num_balls,
        terrain_config,
        seed=sim_config.seed,
    )
    logger.info(f'Generated {len(initial_positions)} initial position(s)')

    final_positions = run_simulation(
        terrain_mesh,
        initial_positions,
        sim_config=sim_config,
        terrain_config=terrain_config,
        gui=args.gui,
    )
    logger.info(f'Received {len(final_positions)} final position(s)')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
