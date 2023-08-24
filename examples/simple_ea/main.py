"""
Optimize a neural network to calculate XOR, using an evolutionary algorithm.

You learn:
- How to create a simple evolutionary loop.
- Basic use of logging.
- Use of the random number generator and the importance of the random seed.
"""

import logging

import config
import numpy as np
import numpy.typing as npt
from evaluate import evaluate
from genotype import Genotype
from individual import Individual
from revolve2.core.optimization.ea import population_management, selection
from revolve2.standard_resources.logging import setup_logging
from revolve2.standard_resources.rng import make_rng, seed_from_string


def select_parents(
    rng: np.random.Generator,
    population: list[Individual],
    offspring_size: int,
) -> npt.NDArray[np.float_]:
    """
    Select pairs of parents using a tournament.

    :param rng: Random number generator.
    :param population: The population to select from.
    :param offspring_size: The number of parent pairs to select.
    :returns: Pairs of indices of selected parents. offspring_size x 2 ints.
    """
    return np.array(
        [
            selection.multiple_unique(
                2,
                [individual.genotype for individual in population],
                [individual.fitness for individual in population],
                lambda _, fitnesses: selection.tournament(rng, fitnesses, k=1),
            )
            for _ in range(offspring_size)
        ],
    )


def select_survivors(
    rng: np.random.Generator,
    original_population: list[Individual],
    offspring_population: list[Individual],
) -> list[Individual]:
    """
    Select survivors using a tournament.

    :param rng: Random number generator.
    :param original_population: The population the parents come from.
    :param offspring_population: The offspring.
    :returns: A newly created population.
    """
    original_survivors, offspring_survivors = population_management.steady_state(
        [i.genotype for i in original_population],
        [i.fitness for i in original_population],
        [i.genotype for i in offspring_population],
        [i.fitness for i in offspring_population],
        lambda n, genotypes, fitnesses: selection.multiple_unique(
            n,
            genotypes,
            fitnesses,
            lambda _, fitnesses: selection.tournament(rng, fitnesses, k=2),
        ),
    )

    return [
        Individual(
            original_population[i].genotype,
            original_population[i].fitness,
        )
        for i in original_survivors
    ] + [
        Individual(
            offspring_population[i].genotype,
            offspring_population[i].fitness,
        )
        for i in offspring_survivors
    ]


def main() -> None:
    """Run the program."""
    # Set up standard logging.
    # This decides the level of severity of logged messages we want to display.
    # By default this is 'INFO' or more severe, and 'DEBUG' is excluded.
    # Furthermore, a standard message layout is set up.
    # If logging is not set up, important messages can be missed.
    setup_logging()

    # Next, set up the random number generator (rng).
    # A rng is not actually random; its randomness comes from an initial source.
    # This is what we call the 'seed'.
    # If you use the same seed, the rng will produce the exact same random numbers.
    # This is how you can make experiments reproducible.
    # It is important that you use a single rng instance in your program.
    # That means you never use the globally defined random number generators, but always the one defined below.\
    # If you don't, your experiment is not reproducible.
    # Reproducibility also means that if you want to do multiple repetitions, or runs, of the same experiment,
    # they must use different seeds.
    # If you don't, your seperate runs will have the same results.

    # First define the seed as a string, unique to your experiment and run.
    # The first part of the string is a number that chose by hand (RNG_SEED).
    # It identifies your experiment.
    # This should be unique within your research, so you never create the same rng between two seperate experiments.
    # The second part identifies the current run of the experiment.
    # In this simple example we have only one run that we call run 0.
    # If you have varying hyper parameters in your experiment they should also be included in this seed.
    run = 0
    seed_as_string = f"experiment_seed_{config.RNG_SEED}_run{run}"
    # Convert the seed to an int.
    final_rng_seed = seed_from_string(seed_as_string)
    rng = np.random.Generator(np.random.PCG64(final_rng_seed))

    # Create an initial population.
    # This is also where we print our first log message.
    # We use the 'info' function to give the message the 'INFO' severity.
    logging.info("Generating initial population.")
    initial_genotypes = [
        Genotype.random(
            rng=rng,
        )
        for _ in range(config.POPULATION_SIZE)
    ]

    # Evaluate the initial population.
    logging.info("Evaluating initial population.")
    initial_fitnesses = [
        evaluate(genotype.parameters) for genotype in initial_genotypes
    ]

    # Create a population of individuals, combining genotype with fitness.
    population = [
        Individual(genotype, fitness)
        for genotype, fitness in zip(initial_genotypes, initial_fitnesses)
    ]

    # Set the current generation to 0.
    generation_index = 0

    # Start the actual optimization process.
    logging.info("Start optimization process.")
    while generation_index < config.NUM_GENERATIONS:
        logging.info(f"Generation {generation_index + 1} / {config.NUM_GENERATIONS}.")

        # Create offspring.
        parents = select_parents(rng, population, config.OFFSPRING_SIZE)
        offspring_genotypes = [
            Genotype.crossover(
                population[parent1_i].genotype,
                population[parent2_i].genotype,
                rng,
            ).mutate(rng)
            for parent1_i, parent2_i in parents
        ]

        # Evaluate the offspring.
        offspring_fitnesses = [
            evaluate(genotype.parameters) for genotype in offspring_genotypes
        ]

        logging.info(f"Max fitness: {max(offspring_fitnesses)}")

        # Make an intermediate offspring population.
        offspring_population = [
            Individual(genotype, fitness)
            for genotype, fitness in zip(offspring_genotypes, offspring_fitnesses)
        ]

        # Create the next generation by selecting survivors between original population and offspring.
        population = select_survivors(
            rng,
            population,
            offspring_population,
        )

        # Increase the generation index counter.
        generation_index += 1


if __name__ == "__main__":
    main()
