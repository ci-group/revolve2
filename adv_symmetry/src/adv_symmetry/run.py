import argparse
import json

from typing import Dict, List

from adv_symmetry.ca_run import run_experiment
from adv_symmetry.argument_schema import parse_json, Arguments


def parse_arguments(args: List[str]) -> Dict:
    parser = argparse.ArgumentParser(
        prog="adv_symmetry",
        description="The main cli for the Advantages of Symmetry Evolutionary Computing project",
    )
    parser.add_argument(
        "json", help="The Json to load the input from", type=argparse.FileType("r")
    )
    arguments = parser.parse_args()

    return json.load(arguments.json)


def run(args: Arguments) -> None:
    run_experiment(
        num_generations=args.num_generations, num_individuals=args.num_individuals
    )


def main(args: List[str]) -> None:
    run(parse_json(parse_arguments(args)))
