from __future__ import annotations

import argparse
from dataclasses import dataclass


@dataclass
class TerrainConfig:
    width: int
    height: float
    resolution: int
    scale: float
    octaves: int
    persistence: float
    lacunarity: float
    filter_size: int

    @classmethod
    def add_argument_group(cls, parser: argparse.ArgumentParser) -> None:
        group = parser.add_argument_group('Terrain Parameters')
        group.add_argument(
            '--width',
            type=int,
            default=20,
            help='terrain width (and length) in meters',
        )
        group.add_argument(
            '--height',
            type=float,
            default=3,
            help='terrain max height in meters',
        )
        group.add_argument(
            '--resolution',
            type=int,
            default=10,
            help='terrain resolution (pixels per meter)',
        )
        group.add_argument(
            '--scale',
            type=float,
            default=10.0,
            help='noise map scale (how far away the map is viewed from)',
        )
        group.add_argument(
            '--octaves',
            type=int,
            default=3,
            help='level of detail in noise map',
        )
        group.add_argument(
            '--persistence',
            type=float,
            default=0.2,
            help='contribution of each octave to map (adjusts amplitude)',
        )
        group.add_argument(
            '--lacunarity',
            type=float,
            default=2.0,
            help=(
                'amount of detail added/removed at each octave '
                '(adjusts frequency)'
            ),
        )
        group.add_argument(
            '--filter-size',
            type=int,
            default=2,
            help='filter size in pixels used to smooth generated terrain',
        )

    @classmethod
    def from_args(cls, args: argparse.Namespace) -> TerrainConfig:
        return TerrainConfig(
            width=args.width,
            height=args.height,
            resolution=args.resolution,
            scale=args.scale,
            octaves=args.octaves,
            persistence=args.persistence,
            lacunarity=args.lacunarity,
            filter_size=args.filter_size,
        )


@dataclass
class SimulationConfig:
    ball_diameter: float
    ball_mass: float
    tick_rate: int
    total_time: int
    seed: int | None

    @classmethod
    def add_argument_group(cls, parser: argparse.ArgumentParser) -> None:
        group = parser.add_argument_group('Simulation Parameters')
        group.add_argument(
            '--ball-diameter',
            type=float,
            default=0.2,
            help='ball diameter in meters',
        )
        group.add_argument(
            '--ball-mass',
            type=float,
            default=0.05,
            help='ball mass in kilograms',
        )
        group.add_argument(
            '--tick-rate',
            type=int,
            default=240,
            help='simulation steps per second',
        )
        group.add_argument(
            '--total-time',
            type=int,
            default=10,
            help='total number of seconds to run the simulation for',
        )
        group.add_argument('--seed', type=int, help='random seed')

    @classmethod
    def from_args(cls, args: argparse.Namespace) -> SimulationConfig:
        return SimulationConfig(
            ball_diameter=args.ball_diameter,
            ball_mass=args.ball_mass,
            tick_rate=args.tick_rate,
            total_time=args.total_time,
            seed=args.seed,
        )
