"""Main script for the example."""

import logging

import multineat
import numpy as np
from sqlalchemy.engine import Engine
from sqlalchemy.orm import Session
import sys
from revolve2.experimentation.evolution.abstract_elements import Evaluator
from ..data_structures import Base, Experiment, Genotype, Generation, Individual, Population
from . import config

from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.optimization.ea import population_management, selection
from revolve2.experimentation.rng import make_rng, seed_from_time
from revolve2.ci_group.morphological_novelty_metric import get_novelty_from_population
from ..tasks import EvaluatorObjectManipulation, EvaluatorLocomotion, EvaluatorSearch


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
    ages = [None] * offspring_size

    i = 0

    for individual in population.individuals:
        if rng.choice([True, False]):
            """Mutate body."""
            offspring_genotypes[i] = individual.genotype.mutate(innov_db_body=innov_db_body,
                                                                innov_db_brain=innov_db_brain, rng=rng)
            ages[i] = generation_index + 1
        else:
            """Only mutate brain and as such, we dont change age."""
            brain = individual.genotype.mutate_brain(innov_db=innov_db_brain, rng=rng)
            offspring_genotypes[i] = Genotype(body=individual.genotype.body, brain=brain.brain)
            ages[i] = individual.age
        i += 1

    offspring_robots = [genotype.develop() for genotype in offspring_genotypes]
    offspring_fitnesses = evaluator.evaluate(offspring_robots)
    offspring_novelty = get_novelty_from_population(offspring_robots)

    # Make an intermediate offspring population.
    offspring_population = Population(
        individuals=[
            Individual(genotype=genotype, fitness=fitness, age=age, novelty=novelty)
            for genotype, fitness, age, novelty in
            zip(offspring_genotypes, offspring_fitnesses, ages, offspring_novelty)
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
    original_survivors, offspring_survivors = population_management.steady_state_morphological_innovation_protection(
        [i.genotype for i in original_population.individuals],
        [i.fitness for i in original_population.individuals],
        [i.age for i in original_population.individuals],
        [i.genotype for i in offspring_population.individuals],
        [i.fitness for i in offspring_population.individuals],
        [i.age for i in offspring_population.individuals],
        lambda values, orders, n: selection.pareto_frontier(
            values,
            orders,
            n,
        ),
    )

    return Population(
        individuals=[
                        Individual(
                            genotype=original_population.individuals[i].genotype,
                            fitness=original_population.individuals[i].fitness,
                            age=original_population.individuals[i].age,
                            novelty=original_population.individuals[i].novelty,
                        )
                        for i in original_survivors
                    ]
                    + [
                        Individual(
                            genotype=offspring_population.individuals[i].genotype,
                            fitness=offspring_population.individuals[i].fitness,
                            age=offspring_population.individuals[i].age,
                            novelty=offspring_population.individuals[i].novelty,
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


def run_experiment(dbengine: Engine, evaluator: Evaluator) -> None:
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
    initial_robots = [genotype.develop() for genotype in initial_genotypes]
    initial_fitness = evaluator.evaluate(initial_robots)
    initial_novelty = get_novelty_from_population(initial_robots)

    # Create a population of individuals, combining genotype with fitness.
    population = Population(
        individuals=[
            Individual(genotype=genotype, fitness=fitness, age=0, novelty=novelty)
            for genotype, fitness, novelty in zip(
                initial_genotypes, initial_fitness, initial_novelty, strict=True
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

        generation = Generation(
            experiment=experiment,
            generation_index=generation.generation_index + 1,
            population=population,
        )

        logging.info("Saving generation.")
        with Session(dbengine, expire_on_commit=False) as session:
            session.add(generation)
            session.commit()


def main(objective: str) -> None:
    """Run the program."""
    # Set up logging.
    setup_logging(file_name="mip_log.txt")

    # Open the database, only if it does not already exist.
    dbengine = open_database_sqlite(
        f"{f'_{objective}.'.join(config.DATABASE_FILE.split('.'))}", open_method=OpenMethod.NOT_EXISTS_AND_CREATE
    )
    # Create the structure of the database.
    Base.metadata.create_all(dbengine)

    # Run the experiment several times.
    for _ in range(config.NUM_REPETITIONS):
        # Intialize the evaluator that will be used to evaluate robots.
        match objective:
            case "l":
                evaluator = EvaluatorLocomotion(headless=True, num_simulators=config.NUM_SIMULATORS)
            case "s":
                evaluator = EvaluatorSearch(headless=True, num_simulators=config.NUM_SIMULATORS)
            case "o":
                evaluator = EvaluatorObjectManipulation(headless=True, num_simulators=config.NUM_SIMULATORS)
            case _:
                raise ValueError(f"Unrecognized objective: {objective}")
        run_experiment(dbengine, evaluator)


if __name__ == "__main__":
    sys.path.append("..")
    objective = sys.argv[1]
    main(objective)
