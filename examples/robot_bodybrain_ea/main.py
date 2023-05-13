"""
Run the example.

A modular robot body and brain will be optimized using a simple evolutionary algorithm.
The genotypes for both body and brain are CPPNWIN.
"""

import hashlib
import logging
from typing import List, Tuple, cast

import config
import multineat
import numpy as np
from base import Base
from evaluator import Evaluator
from generation import Generation
from genotype import Genotype
from individual import Individual
from population import Population
from revolve2.core.database import OpenMethod, open_database_sqlite
from revolve2.core.optimization.ea import population_management, selection
from sqlalchemy.orm import Session


def select_parents(
    rng: np.random.Generator,
    population: Population,
    offspring_size: int,
) -> List[Tuple[int, int]]:
    """
    Select pairs of parents using a tournament.

    :param rng: Random number generator.
    :param population: The population to select from.
    :param offspring_size: The number of parent pairs to select.
    :returns: Pairs of indices of selected parents.
    """
    return [
        cast(
            Tuple[int, int],
            selection.multiple_unique(
                2,
                [individual.genotype for individual in population.individuals],
                [individual.fitness for individual in population.individuals],
                lambda _, fitnesses: selection.tournament(rng, fitnesses, k=1),
            ),
        )
        for _ in range(offspring_size)
    ]


def select_survivors(
    rng: np.random.Generator,
    original_population: Population,
    offspring_population: Population,
) -> Population:
    """
    Select survivors using a tournament.

    :param rng: Random number generator.
    :param original_population: The population the parents come from.
    :param offspring_population: The offspring.
    :returns: A newly created population.
    """
    original_survivors, offspring_survivors = population_management.steady_state(
        [i.genotype for i in original_population.individuals],
        [i.fitness for i in original_population.individuals],
        [i.genotype for i in offspring_population.individuals],
        [i.fitness for i in offspring_population.individuals],
        lambda n, genotypes, fitnesses: selection.multiple_unique(
            n,
            genotypes,
            fitnesses,
            lambda _, fitnesses: selection.tournament(rng, fitnesses, k=2),
        ),
    )

    return Population(
        [
            Individual(
                original_population.individuals[i].genotype,
                original_population.individuals[i].fitness,
            )
            for i in original_survivors
        ]
        + [
            Individual(
                offspring_population.individuals[i].genotype,
                offspring_population.individuals[i].fitness,
            )
            for i in offspring_survivors
        ]
    )


def main() -> None:
    """Run the program."""
    # set up logging we see all relevant logging messages
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    # create a unique seed and initialize the random number generator
    rng_seed = int(
        hashlib.sha256(
            f"robot_bodybrain_ea_seed{config.RNG_SEED}".encode()
        ).hexdigest(),
        16,
    )
    rng = np.random.Generator(np.random.PCG64(rng_seed))

    # intialize the evaluator that will be used to evaluate robots
    evaluator = Evaluator(
        headless=True,
        num_simulators=config.NUM_SIMULATORS,
        simulation_time=config.SIMULATION_TIME,
        sampling_frequency=config.SAMPLING_FREQUENCY,
        control_frequency=config.CONTROL_FREQUENCY,
    )

    # multineat innovation databases
    innov_db_body = multineat.InnovationDatabase()
    innov_db_brain = multineat.InnovationDatabase()

    # open the database
    dbengine = open_database_sqlite(
        config.DATABASE_FILE,
        open_method=OpenMethod.OVERWITE_IF_EXISTS,  # TODO change to not exists after development
    )
    Base.metadata.create_all(dbengine)

    # create and measure the initial population
    logging.info("Generating initial population.")
    initial_genotypes = [
        Genotype.random(
            innov_db_body=innov_db_body,
            innov_db_brain=innov_db_brain,
            rng=rng,
        )
        for _ in range(config.POPULATION_SIZE)
    ]
    logging.info("Evaluating initial population.")
    initial_fitnesses = evaluator.evaluate(
        [genotype.develop() for genotype in initial_genotypes]
    )
    population = Population(
        [
            Individual(genotype, fitness)
            for genotype, fitness in zip(initial_genotypes, initial_fitnesses)
        ]
    )

    # make it into a generation
    generation = Generation(
        0,
        population,
    )

    # save the initial generation
    logging.info("Saving initial generation.")
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
            ).mutate(innov_db_body, innov_db_brain, rng)
            for parent1_i, parent2_i in parents
        ]

        # evaluate the offspring
        offspring_fitnesses = evaluator.evaluate(
            [genotype.develop() for genotype in offspring_genotypes]
        )

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
