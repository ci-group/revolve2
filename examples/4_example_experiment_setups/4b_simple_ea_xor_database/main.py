"""Main script for the example."""

import logging
from typing import Any

import config
import numpy as np
from database_components import (
    Base,
    Experiment,
    Generation,
    Genotype,
    Individual,
    Population,
)
from evaluate import Evaluator
from numpy.typing import NDArray
from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.evolution.abstract_elements import Reproducer, Selector
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.optimization.ea import population_management, selection
from revolve2.experimentation.rng import make_rng, make_rng_time_seed, seed_from_time
from sqlalchemy.engine import Engine
from sqlalchemy.orm import Session


class ParentSelector(Selector):
    """Here we create a selector object that helps us select the parents for reproduction."""

    _rng: np.random.Generator
    _offspring_size: int

    def __init__(self, offspring_size: int) -> None:
        """
        Initialize the ParentSelector object.

        :param offspring_size: The size of the offspring for selection.
        """
        self._rng = make_rng_time_seed()
        self._offspring_size = offspring_size

    def select(
        self, population: Population, **kwargs: Any
    ) -> tuple[NDArray[np.int_], dict[str, Population]]:
        """
        Select pairs of parents using a tournament.

        :param population: The population to select from.
        :param kwargs: Additional kwargs that are not used in this example.
        :returns: Pairs of indices of selected parents. offspring_size x 2 ints, and the parent population in the KWArgs dict.
        """
        return np.asarray(
            [
                selection.multiple_unique(
                    2,
                    [individual.genotype for individual in population.individuals],
                    [individual.fitness for individual in population.individuals],
                    lambda _, fitnesses: selection.tournament(
                        self._rng, fitnesses, k=1
                    ),
                )
                for _ in range(self._offspring_size)
            ]
        ), {"parent_population": population}


class SurvivorSelector(Selector):
    """Here we make a selector object that allows us to select survivor after evaluation."""

    _rng: np.random.Generator

    def __init__(self) -> None:
        """Initialize the selector."""
        self._rng = make_rng_time_seed()

    def select(
        self, population: Population, **kwargs: Any
    ) -> tuple[Population, dict[Any, Any]]:
        """
        Select survivors using a tournament.

        :param population: The initial population.
        :param kwargs: Additional kwargs that contain the children to do selection with.
        :returns: The selected population and empty kwargs in this implementation.
        :raises KeyError: If no children got passed.
        """
        offspring: list[Individual] | None = kwargs.get("children")
        if offspring is None:
            raise KeyError("No children passed.")
        original_survivors, offspring_survivors = population_management.steady_state(
            [i.genotype for i in population.individuals],
            [i.fitness for i in population.individuals],
            [i.genotype for i in offspring],
            [i.fitness for i in offspring],
            lambda n, genotypes, fitnesses: selection.multiple_unique(
                n,
                genotypes,
                fitnesses,
                lambda _, fitnesses: selection.tournament(self._rng, fitnesses, k=2),
            ),
        )

        return (
            Population(
                individuals=[
                    Individual(
                        genotype=population.individuals[i].genotype,
                        fitness=population.individuals[i].fitness,
                    )
                    for i in original_survivors
                ]
                + [
                    Individual(
                        genotype=offspring[i].genotype,
                        fitness=offspring[i].fitness,
                    )
                    for i in offspring_survivors
                ]
            ),
            {},
        )


class CrossoverReproducer(Reproducer):
    """We make a reproducer object to facilitate crossover operations."""

    _rng: np.random.Generator

    def __init__(self) -> None:
        """Initialize the Reproducer."""
        self._rng = make_rng_time_seed()

    def reproduce(self, population: NDArray[np.int_], **kwargs: Any) -> list[Genotype]:
        """
        Make Individuals Reproduce.

        :param population: The population.
        :param kwargs: Additional arguments.
        :returns: The reproduced population.
        :raises KeyError: If the parents are not passed.
        """
        parents: list[Individual] | None = kwargs.get(
            "parent_population"
        )  # We select the population of parents that were passed in KWArgs of the parent selector object.
        if parents is None:
            raise KeyError("No children passed.")
        offspring = [
            Genotype.crossover(
                parents[parent1_i].genotype,
                parents[parent2_i].genotype,
                self._rng,
                num_parameters=config.NUM_PARAMETERS,
            ).mutate(
                self._rng,
                mutate_std=config.MUTATE_STD,
                num_parameters=config.NUM_PARAMETERS,
            )
            for parent1_i, parent2_i in population
        ]
        return offspring


def run_experiment(dbengine: Engine) -> None:
    """
    Run an experiment.

    :param dbengine: An openened database with matching initialize database structure.
    """
    logging.info("----------------")
    logging.info("Start experiment")

    # Set up the random number generator.
    # We are manually generating the seed, because we are going to save it in our database.
    rng_seed = seed_from_time()
    rng = make_rng(rng_seed)

    # Create a new experiment instance.
    experiment = Experiment(rng_seed=rng_seed)

    """
    Now we save it to the database.
    A session manages multiple changes to a database, which then can either be committed(accepted) or a rolled back(aborted) in case something bad happens in our code.
    We add the experiment and commit as nothing can go wrong.
    """
    logging.info("Saving experiment configuration.")
    with Session(dbengine) as session:
        session.add(experiment)
        session.commit()

    # Create an initial population.
    logging.info("Generating initial population.")
    initial_genotypes = [
        Genotype.random(rng=rng, num_parameters=config.NUM_PARAMETERS)
        for _ in range(config.POPULATION_SIZE)
    ]
    # Here we instantiate an evaluator, that allows us to evaluate a population of solutions.
    evaluator = Evaluator()
    # Now we instantiate a selector object to do parent selection.
    parent_selector = ParentSelector(offspring_size=config.OFFSPRING_SIZE)
    # We do the same to make survivor selection possible.
    survivor_selector = SurvivorSelector()
    # We also make a reproducer that allows us to get new individuals from our parents.
    reproducer = CrossoverReproducer()

    # Evaluate the initial population.
    logging.info("Evaluating initial population.")
    initial_fitnesses = evaluator.evaluate(initial_genotypes)

    # Create a population of individuals, combining genotype with fitness.
    population = Population(
        individuals=[
            Individual(genotype=genotype, fitness=fitness)
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
        parent_pairs, parent_kwargs = parent_selector.select(population)
        offspring_genotypes = reproducer.reproduce(
            parent_pairs, **parent_kwargs
        )  # we pass the parent pairs and the kwargs.

        # Evaluate the offspring.
        offspring_fitnesses = evaluator.evaluate(offspring_genotypes)

        # Make an intermediate offspring population.
        offspring_population = Population(
            individuals=[
                Individual(genotype=genotype, fitness=fitness)
                for genotype, fitness in zip(offspring_genotypes, offspring_fitnesses)
            ]
        )

        # Create the next generation by selecting survivors between original population and offspring.
        population, _ = survivor_selector.select(
            population,
            children=offspring_population,
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

    """
    We open the database, only if it does not already exist.
    If it did something when wrong in a previous run of this program, and we must manually figure out what to do with the existing database.
    (maybe throw away?)
    """
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
