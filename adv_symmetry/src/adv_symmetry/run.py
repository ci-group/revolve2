import argparse
from dataclasses import dataclass

from typing import List

from adv_symmetry.experiment import run_experiment


@dataclass
class Args:
    pass


def parse_arguments(args: List[str]) -> Args:
    parser = argparse.ArgumentParser(
        prog="adv_symmetry",
        description="The main cli for the Advantages of Symmetry Evolutionary Computing project",
    )
    return Args()


def run(args: Args) -> None:
    run_experiment()


def main(args: List[str]) -> None:
    run(parse_arguments(args))
