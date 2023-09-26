import logging

import cma
import config
from evaluator import Evaluator
from revolve2.ci_group.logging import setup_logging
from revolve2.ci_group.rng import seed_from_time
from revolve2.modular_robot.brains import (
    body_to_actor_and_cpg_network_structure_neighbour,
)


def main() -> None:
    """Run the experiment."""
    setup_logging(file_name="log.txt")

    # Get the actor and cpg network structure for the body of choice.
    # The cpg network structure describes the connections between neurons in the cpg brain.
    _, cpg_network_structure = body_to_actor_and_cpg_network_structure_neighbour(
        config.BODY
    )

    # Intialize the evaluator that will be used to evaluate robots.
    evaluator = Evaluator(
        headless=True,
        num_simulators=config.NUM_SIMULATORS,
        cpg_network_structure=cpg_network_structure,
        body=config.BODY,
    )

    # Initial parameter values for the brain.
    initial_mean = cpg_network_structure.num_connections * [0.5]

    # We use the CMA-ES optimizer from the cma python package.
    # Initialize the cma optimizer.
    options = cma.CMAOptions()
    options.set("bounds", [-1.0, 1.0])
    # The cma package uses its own internal rng.
    # Instead of creating our own numpy rng, we use our seed to initialize cma.
    rng_seed = seed_from_time() % 2**32  # Cma seed must be smaller than 2**32.
    options.set("seed", rng_seed)
    opt = cma.CMAEvolutionStrategy(initial_mean, config.INITIAL_STD, options)

    generation_index = 0

    # Run cma for the defined number of generations.
    logging.info("Start optimization process.")
    while generation_index < config.NUM_GENERATIONS:
        logging.info(f"Generation {generation_index + 1} / {config.NUM_GENERATIONS}.")

        # Get the sampled solutions(parameters) from cma.
        solutions = opt.ask()

        # Evaluate them. Invert because fitness maximizes, but cma minimizes.
        fitnesses = -evaluator.evaluate(solutions)

        # Tell cma the fitnesses.
        opt.tell(solutions, fitnesses)

        logging.info(f"{opt.result.xbest=} {opt.result.fbest=}")

        # Increase the generation index counter.
        generation_index += 1


if __name__ == "__main__":
    main()
