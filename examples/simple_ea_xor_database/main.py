"""
This example adds a database to the 'simple_ea_xor' example.

Naturally, look at that example first.

You learn:
- How to save intermediate and final results to a database using the SQLAlchemy ORM.

Beforehand you should have:
- A basic understanding of relational databases.
- An understanding of dataclasses is beneficial.

Introduction:
In the previous example, all results were written to the console.
We would like to store all interesting data we come across in a more structured way.
There are many options, such as saving text to files, various kinds of databases, or others such as the 'weights and biases' service.
Here we choose to use the SQLite database.
This a local relational database stored in a single file that is designed to be as simple as possible.
It is very beneficial to have a basic understanding of relational databases, as this tutorial will not explain them in-depth.

To simplify working with this database, we will use to 'object-relational mapping' (ORM) library 'SQLAlchemy'.
This software lets you declare the database structure in Python, simply by creating classes similar to Python's 'dataclass'.
If you do not know what a dataclass is, it is worth looking that up.
However, it will probably also become apparent how SQLAlchemy works without that knowledge.
The main addition in this tutorial is that some classes are changed or added to follow this ORM description.
If at any point you do not understand something related to SQLAlchemy,
be aware that it is a third-party library with an extensive documentation available to you online.
"""

import logging

import config
import numpy as np
import numpy.typing as npt
from base import Base
from evaluate import evaluate
from experiment import Experiment
from generation import Generation
from genotype import Genotype
from individual import Individual
from population import Population
from revolve2.ci_group.logging import setup_logging
from revolve2.ci_group.rng import make_rng, seed_from_time
from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.optimization.ea import population_management, selection
from sqlalchemy.engine import Engine
from sqlalchemy.orm import Session


def select_parents(
    rng: np.random.Generator,
    population: Population,
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
                [individual.genotype for individual in population.individuals],
                [individual.fitness for individual in population.individuals],
                lambda _, fitnesses: selection.tournament(rng, fitnesses, k=1),
            )
            for _ in range(offspring_size)
        ],
    )


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


def run_experiment(dbengine: Engine) -> None:
    """
    Run an experiment.

    :param dbengine: An openened database with matching initialize database structure.
    """
    logging.info("----------------")
    logging.info("Start experiment")

    # Set up the random number generater.
    # We are manually generating the seed, because we are going to save it in our database.
    rng_seed = seed_from_time()
    rng = make_rng(rng_seed)

    # Create a new experiment instance.
    experiment = Experiment(rng_seed=rng_seed)

    # Save it to the database.
    # A session manages multiple changes to a database,
    # which then can either be committed(accepted) or a rolled back(aborted) in case something bad happens in our code.
    # We add the experiment and commit as nothing can go wrong.
    logging.info("Saving experiment configuration.")
    with Session(dbengine) as session:
        session.add(experiment)
        session.commit()

    # Create an initial population.
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
    population = Population(
        [
            Individual(genotype, fitness)
            for genotype, fitness in zip(initial_genotypes, initial_fitnesses)
        ]
    )

    # Finish the zeroth generation and save it to the database.
    # The generation references the experiment so we later know which experiment it was part of.
    generation = Generation(
        experiment=experiment, generation_index=0, population=population
    )
    logging.info("Saving generation.")
    with Session(dbengine, expire_on_commit=False) as session:
        session.add(generation)
        session.commit()

    # Start the actual optimization process.
    logging.info("Start optimization process.")
    while generation.generation_index < config.NUM_GENERATIONS:
        logging.info(
            f"Generation {generation.generation_index + 1} / {config.NUM_GENERATIONS}."
        )

        # Create offspring.
        parents = select_parents(rng, population, config.OFFSPRING_SIZE)
        offspring_genotypes = [
            Genotype.crossover(
                population.individuals[parent1_i].genotype,
                population.individuals[parent2_i].genotype,
                rng,
            ).mutate(rng)
            for parent1_i, parent2_i in parents
        ]

        # Evaluate the offspring.
        offspring_fitnesses = [
            evaluate(genotype.parameters) for genotype in offspring_genotypes
        ]

        # Make an intermediate offspring population.
        offspring_population = Population(
            [
                Individual(genotype, fitness)
                for genotype, fitness in zip(offspring_genotypes, offspring_fitnesses)
            ]
        )

        # Create the next generation by selecting survivors between original population and offspring.
        population = select_survivors(
            rng,
            population,
            offspring_population,
        )

        # Make it all into a generation and save it to the database.
        generation = Generation(
            experiment=experiment,
            generation_index=generation.generation_index + 1,
            population=population,
        )
        logging.info("Saving generation.")
        with Session(dbengine, expire_on_commit=False) as session:
            session.add(generation)
            session.commit()


def main() -> None:
    """Run the program."""
    # Set up standard logging.
    setup_logging(file_name="log.txt")

    # Open the database, only if it does not already exist.
    # If it did something when wrong in a previous run of this program,
    # and we must manually figure out what to do with the existing database.
    # (maybe throw away?)
    dbengine = open_database_sqlite(
        config.DATABASE_FILE, open_method=OpenMethod.NOT_EXISTS_AND_CREATE
    )
    # Create the structure of the database.
    # Take a look at the 'Base' class.
    Base.metadata.create_all(dbengine)

    # We are running several repetitions of the same experiment.
    for _ in range(config.NUM_REPETITIONS):
        run_experiment(dbengine)


if __name__ == "__main__":
    main()
