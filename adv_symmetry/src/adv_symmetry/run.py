import argparse
from dataclasses import dataclass

from typing import List

from adv_symmetry.ca_run import run_experiment


@dataclass
class Args:
    num_generations: int
    num_individuals: int


def parse_arguments(args: List[str]) -> Args:
    parser = argparse.ArgumentParser(
        prog="adv_symmetry",
        description="The main cli for the Advantages of Symmetry Evolutionary Computing project",
    )
    parser.add_argument(
        "num_generations", help="The amount of generations to run the GA for", type=int
    )
    parser.add_argument(
        "num_individuals", help="The amount of individuals in each generation", type=int
    )
    arguments = parser.parse_args()
    return Args(arguments.num_generations, arguments.num_individuals)


def run(args: Args) -> None:
    run_experiment(
        num_generations=args.num_generations, num_individuals=args.num_individuals
    )


def main(args: List[str]) -> None:
    run(parse_arguments(args))
