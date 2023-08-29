"""
Set up an experiment that optimizes the brain of a given robot body using CMA-ES.

As the body is static, the genotype of the brain will be a fixed length real-valued vector.

Before starting this tutorial, it is useful to look at the 'experiment_setup' and 'evaluate_multiple_isolated_robots' examples.
It is also nice to understand the concept of a cpg brain, although not really needed.

You learn:
- How to optimize the brain of a robot using CMA-ES.
"""

import logging

import cma
import config
from evaluator import Evaluator
from revolve2.core.modular_robot.brains import (
    body_to_actor_and_cpg_network_structure_neighbour,
)
from revolve2.standard_resources.logging import setup_logging
from revolve2.standard_resources.rng import seed_from_time


def main() -> None:
    """Run the experiment."""
    setup_logging(file_name="log.txt")

    # Get the actor and cpg network structure for the body of choice
    # The cpg network structure describes the connections between neurons in the cpg brain.
    _, cpg_network_structure = body_to_actor_and_cpg_network_structure_neighbour(
        config.BODY
    )

    # Intialize the evaluator that will be used to evaluate robots
    evaluator = Evaluator(
        headless=True,
        num_simulators=config.NUM_SIMULATORS,
        cpg_network_structure=cpg_network_structure,
        body=config.BODY,
    )

    # Initial parameter values for the brain
    initial_mean = cpg_network_structure.num_connections * [0.5]

    # We use the CMA-ES optimizer from the cma python package.
    # Initialize the cma optimizer
    options = cma.CMAOptions()
    options.set("bounds", [-1.0, 1.0])
    # The cma package uses its own internal rng.
    # Instead of creating our own numpy rng, we use our seed to initialize cma.
    rng_seed_int = seed_from_time() % 2**32  # Cma seed must be smaller than 2**32
    options.set("seed", rng_seed_int)
    opt = cma.CMAEvolutionStrategy(initial_mean, config.INITIAL_STD, options)

    generation_index = 0

    # Run cma for the defined number of generations
    logging.info("Start optimization process.")
    while generation_index < config.NUM_GENERATIONS:
        logging.info(f"Generation {generation_index + 1} / {config.NUM_GENERATIONS}.")

        # Get the sampled solutions(parameters) from cma
        solutions = opt.ask()

        # Evaluate them. Invert because fitness maximizes, but cma minimizes
        fitnesses = -evaluator.evaluate(solutions)

        # Tell cma the fitnesses
        opt.tell(solutions, fitnesses)

        logging.info(f"{opt.result.xbest=} {opt.result.fbest=}")

        # Increase the generation index counter.
        generation_index += 1


if __name__ == "__main__":
    main()
