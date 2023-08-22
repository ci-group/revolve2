"""
Optimize a neural network to calculate XOR, using an evolutionary algorithm.

You learn:
- How to create a simple evolutionary loop.
- Basic use of logging.
"""

import logging
import numpy as np
import config
from genotype import Genotype
from revolve2.standard_resources.logging import setup_logging
from parameters import Parameters

def evaluate_network(params: Parameters, inputs: np.ndarray[float, 2]) -> float:
    """
    Pass two inputs through a fully connected relu network.

    :param params: The parameters to evaluate.
    :param input1: First input for network.
    :param input2: Second input for network.
    :returns: The output of the network.
    """
    # First layer
    n0 = np.maximum(0, np.dot(params[:2], inputs) + params[2])
    n1 = np.maximum(0, np.dot(params[3:5], inputs) + params[5])
    
    # Second layer
    output = np.maximum(0, n0 * params[6] + n1 * params[7] + params[8])
    
    return output




def evaluate(parameters: Parameters) -> np.ndarray[float, 5]:
    """
    Measure one set of parameters.

    :param parameters: The parameters to measure.
    :returns: Sum of squared errors and each individual error.
    """
    inputs = np.array([[0, 0], [1, 0], [0, 1], [1, 1]])
    expected_outputs = np.array([0,1,1,0])

    outputs = np.array([evaluate_network(parameters, input) for input in inputs])

    return -np.abs(outputs - expected_outputs)**2


def main() -> None:
    """Run the program."""

    # Set up standard logging.
    # This decides the level of severity of logged messages we want to display.
    # By default this is 'INFO' or more severe, and 'DEBUG' is excluded.
    # Furthermore, a standard message layout is set up.
    # If logging is not set up, important messages can be missed.
    setup_logging()

    # Set up the random number generater.
    rng = np.random.Generator(np.random.PCG64(config.RNG_SEED))

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
        evaluate(genotype.parameters)
        for genotype in initial_genotypes
    ]
    population = Population(
        [
            Individual(genotype, fitness)
            for genotype, fitness in zip(initial_genotypes, initial_fitnesses)
        ]
    )

    # create initial generation from initial population
    generation = Generation(
        0,
        population,
    )

    # save the initial generation
    logging.info("Saving initial population.")
    with Session(dbengine, expire_on_commit=False) as ses:
        ses.add(generation)
        ses.commit()

    # below is the actual optimization process
    logging.info("Start optimization process.")
    while generation.generation_index < config.NUM_GENERATIONS:
        logging.info(
            f"Generation {generation.generation_index + 1} / {config.NUM_GENERATIONS}."
        )

        # create offspring
        parents = select_parents(rng, generation.population, config.OFFSPRING_SIZE)
        offspring_genotypes = [
            Genotype.crossover(
                generation.population.individuals[parent1_i].genotype,
                generation.population.individuals[parent2_i].genotype,
                rng,
            ).mutate(rng)
            for parent1_i, parent2_i in parents
        ]

        # evaluate the offspring
        offspring_fitnesses = [
            evaluate(cast(ParamTuple, genotype.parameters))[0]
            for genotype in offspring_genotypes
        ]

        # make an intermediate offspring population
        offspring_population = Population(
            [
                Individual(genotype, fitness)
                for genotype, fitness in zip(offspring_genotypes, offspring_fitnesses)
            ]
        )

        # create the next population by selecting survivors
        survived_population = select_survivors(
            rng,
            generation.population,
            offspring_population,
        )

        # make it into then next generation
        generation = Generation(
            generation.generation_index + 1,
            survived_population,
        )

        # save the newly created generation
        with Session(dbengine, expire_on_commit=False) as ses:
            ses.add(generation)
            ses.commit()


if __name__ == "__main__":
    main()
