"""Main script for the example."""

import logging
from typing import Any

import config
import numpy as np
from evaluate import Evaluator
from genotype import Genotype
from individual import Individual
from numpy.typing import NDArray
from revolve2.experimentation.evolution.abstract_elements import Reproducer, Selector
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.optimization.ea import population_management, selection
from revolve2.experimentation.rng import make_rng_time_seed


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
        self, population: list[Individual], **kwargs: Any
    ) -> tuple[NDArray[np.int_], dict[str, list[Individual]]]:
        """
        Select pairs of parents using a tournament.selection procedure.

        :param population: The population to select from.
        :param kwargs: Additional kwargs that are not used in this example.
        :returns: Pairs of indices of selected parents. offspring_size x 2 ints, and the parent population in the KWArgs dict.
        """
        final_selection = np.asarray(
            [
                selection.multiple_unique(
                    2,
                    [individual.genotype for individual in population],
                    [individual.fitness for individual in population],
                    lambda _, fitnesses: selection.tournament(
                        self._rng, fitnesses, k=1
                    ),
                )
                for _ in range(self._offspring_size)
            ]
        )
        """We select not the parents directly, but their respective indices for the reproduction step."""
        return final_selection, {"parent_population": population}


class SurvivorSelector(Selector):
    """Here we make a selector object that allows us to select survivor after evaluation."""

    _rng: np.random.Generator

    def __init__(self) -> None:
        """Initialize the selector."""
        self._rng = make_rng_time_seed()

    def select(
        self, population: list[Individual], **kwargs: Any
    ) -> tuple[list[Individual], dict[Any, Any]]:
        """
        Select survivors using a tournament selection.

        :param population: The initial population.
        :param kwargs: Additional kwargs that contain the children to do selection with.
        :returns: A newly created population.
        :raises KeyError: If no children got passed.
        """
        offspring: list[Individual] | None = kwargs.get("children")
        if offspring is None:
            raise KeyError("No children passed.")

        """
        We want to get the survivors for the next generation, using our old population and the new children.
        We determine the survivors by using a tournament, in which to random robots 'compete' against each other. 
        This competition is based on the fitness value and the fitter robot will survive.
        """
        original_survivors, offspring_survivors = population_management.steady_state(
            [i.genotype for i in population],
            [i.fitness for i in population],
            [i.genotype for i in offspring],
            [i.fitness for i in offspring],
            lambda n, genotypes, fitnesses: selection.multiple_unique(
                n,
                genotypes,
                fitnesses,
                lambda _, fitnesses: selection.tournament(self._rng, fitnesses, k=2),
            ),
        )

        return [
            Individual(
                genotype=population[i].genotype,
                fitness=population[i].fitness,
            )
            for i in original_survivors
        ] + [
            Individual(
                genotype=offspring[i].genotype,
                fitness=offspring[i].fitness,
            )
            for i in offspring_survivors
        ], {}


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
        :return: The children.
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
            ).mutate(self._rng)
            for parent1_i, parent2_i in population
        ]
        return offspring


def main() -> None:
    """Run the program."""
    # Set up logging.
    setup_logging()
    rng = make_rng_time_seed()

    # Create an initial population.
    logging.info("Generating initial population.")
    initial_genotypes = [
        Genotype.random(
            rng=rng,
        )
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
        parent_pairs, parent_kwargs = parent_selector.select(population)
        offspring_genotypes = reproducer.reproduce(
            parent_pairs, **parent_kwargs
        )  # we pass the parent pairs and the kwargs.

        # Evaluate the offspring.
        offspring_fitnesses = evaluator.evaluate(offspring_genotypes)

        logging.info(f"Max fitness: {max(offspring_fitnesses)}")

        # Make an intermediate offspring population.
        offspring_population = [
            Individual(genotype, fitness)
            for genotype, fitness in zip(offspring_genotypes, offspring_fitnesses)
        ]

        # Create the next generation by selecting survivors between original population and offspring.
        population, _ = survivor_selector.select(
            population,
            children=offspring_population,
        )

        # Increase the generation index counter.
        generation_index += 1


if __name__ == "__main__":
    main()
