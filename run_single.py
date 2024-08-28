from __future__ import annotations

import argparse
import sys
from typing import Sequence

from simulation.config import SimulationConfig
from simulation.config import TerrainConfig
from simulation.simulate import generate_initial_positions
from simulation.simulate import generate_noisemap
from simulation.simulate import generate_vertices
from simulation.simulate import run_simulation


def main(argv: Sequence[str] | None = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description='Simulate a golf ball',
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

    sim_config = SimulationConfig.from_args(args)
    terrain_config = TerrainConfig.from_args(args)

    terrain_heightmap = generate_noisemap(terrain_config)
    terrain_mesh = generate_vertices(terrain_heightmap, terrain_config)

    (initial_position,) = generate_initial_positions(1, terrain_config)
    print(f'Initial position: {initial_position}')

    (final_position,) = run_simulation(
        terrain_mesh,
        [initial_position],
        sim_config=sim_config,
        terrain_config=terrain_config,
        gui=args.gui,
    )
    print(f'Final position: {final_position}')


if __name__ == '__main__':
    raise SystemExit(main())
