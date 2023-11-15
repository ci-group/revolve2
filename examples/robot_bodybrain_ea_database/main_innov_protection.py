"""Main script for the example."""

import logging

import innov_protection_config as config
import multineat
import numpy as np
from base import Base
from evaluator import Evaluator
from experiment import Experiment
from generation import Generation
from genotype import Genotype
from individual import Individual
from population import Population
from sqlalchemy.engine import Engine
from sqlalchemy.orm import Session

from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.optimization.ea import population_management, selection
from revolve2.experimentation.rng import make_rng, seed_from_time

def mutate_parents(
        rng: np.random.Generator,
        population: Population,
        offspring_size: int,
        evaluator: Evaluator,
        innov_db_body: multineat.InnovationDatabase,
        innov_db_brain: multineat.InnovationDatabase,
        generation_index: int,
) -> Population:
    """
    Select pairs of parents using a tournament.

    :param rng: Random number generator.
    :param population: The population to select from.
    :param offspring_size: The number of parent pairs to select.
    :returns: The offspring.
    """
    offspring_genotypes = [None] * offspring_size
    generation = [None] * offspring_size

    i = 0
    args = {
        "innov_db_body": innov_db_body,
        "innov_db_brain": innov_db_brain,
        "rng": rng,
    }

    for individual in population.individuals:
        if rng.choice([True, False]):
            """Mutate body."""
            offspring_genotypes[i] = individual.genotype.mutate(**args)
            generation[i] = generation_index + 1
        else:
            brain = individual.genotype.mutate_brain(**args)
            offspring_genotypes[i] = Genotype(body=individual.genotype.body, brain=brain.brain)
            generation[i] = individual.age
        i += 1

    offspring_fitnesses = evaluator.evaluate(
        [genotype.develop() for genotype in offspring_genotypes]
    )



    # Make an intermediate offspring population.
    offspring_population = Population(
        individuals=[
            Individual(genotype=genotype, fitness=fitness, age=age)
            for genotype, fitness, age in zip(offspring_genotypes, offspring_fitnesses, generation)
        ]
    )
    return offspring_population


def select_survivors(
        original_population: Population,
        offspring_population: Population,
) -> Population:
    """
    Select survivors using a pareto frontier.

    :param original_population: The population the parents come from.
    :param offspring_population: The offspring.
    :returns: A newly created population.
    """
    original_survivors, offspring_survivors = population_management.steady_state_innovation_protection(
        [i.genotype for i in original_population.individuals],
        [i.fitness for i in original_population.individuals],
        [i.age for i in original_population.individuals],
        [i.genotype for i in offspring_population.individuals],
        [i.fitness for i in offspring_population.individuals],
        [i.age for i in offspring_population.individuals],
        lambda genotypes, fitnesses, ages, n: selection.pareto_frontier(
            genotypes,
            fitnesses,
            ages,
            n,
        ),
    )

    return Population(
        individuals=[
                        Individual(
                            genotype=original_population.individuals[i].genotype,
                            fitness=original_population.individuals[i].fitness,
                            age=original_population.individuals[i].age,
                        )
                        for i in original_survivors
                    ]
                    + [
                        Individual(
                            genotype=offspring_population.individuals[i].genotype,
                            fitness=offspring_population.individuals[i].fitness,
                            age=offspring_population.individuals[i].age,
                        )
                        for i in offspring_survivors
                    ]
    )


def find_best_robot(
        current_best: Individual | None, population: list[Individual]
) -> Individual:
    """
    Return the best robot between the population and the current best individual.

    :param current_best: The current best individual.
    :param population: The population.
    :returns: The best individual.
    """
    return max(
        population + [] if current_best is None else [current_best],
        key=lambda x: x.fitness,
    )


def run_experiment(dbengine: Engine) -> None:
    """
    Run an experiment.

    :param dbengine: An openened database with matching initialize database structure.
    """
    logging.info("----------------")
    logging.info("Start experiment")

    # Set up the random number generator.
    rng_seed = seed_from_time()
    rng = make_rng(rng_seed)

    # Create and save the experiment instance.
    experiment = Experiment(rng_seed=rng_seed)
    logging.info("Saving experiment configuration.")
    with Session(dbengine) as session:
        session.add(experiment)
        session.commit()

    # Intialize the evaluator that will be used to evaluate robots.
    evaluator = Evaluator(headless=True, num_simulators=config.NUM_SIMULATORS)

    # CPPN innovation databases.
    innov_db_body = multineat.InnovationDatabase()
    innov_db_brain = multineat.InnovationDatabase()

    # Create an initial population.
    logging.info("Generating initial population.")
    initial_genotypes = [
        Genotype.random(
            innov_db_body=innov_db_body,
            innov_db_brain=innov_db_brain,
            rng=rng,
        )
        for _ in range(config.POPULATION_SIZE)
    ]

    # Evaluate the initial population.
    logging.info("Evaluating initial population.")
    initial_fitnesses = evaluator.evaluate(
        [genotype.develop() for genotype in initial_genotypes]
    )

    # Create a population of individuals, combining genotype with fitness.
    population = Population(
        individuals=[
            Individual(genotype=genotype, fitness=fitness, age=0)
            for genotype, fitness in zip(
                initial_genotypes, initial_fitnesses, strict=True
            )
        ]
    )

    # Finish the zeroth generation and save it to the database.
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
        offspring_population = mutate_parents(
            rng,
            population,
            config.OFFSPRING_SIZE,
            evaluator,
            innov_db_body,
            innov_db_brain,
            generation.generation_index,
        )


        # Create the next population by selecting survivors.
        population = select_survivors(
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
    # Set up logging.
    setup_logging(file_name="log.txt")

    # Open the database, only if it does not already exists.
    dbengine = open_database_sqlite(
        f"innov_protection_{config.DATABASE_FILE}", open_method=OpenMethod.NOT_EXISTS_AND_CREATE
    )
    # Create the structure of the database.
    Base.metadata.create_all(dbengine)

    # Run the experiment several times.
    for _ in range(config.NUM_REPETITIONS):
        run_experiment(dbengine)


if __name__ == "__main__":
    main()
