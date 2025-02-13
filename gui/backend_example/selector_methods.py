from revolve2.experimentation.evolution.abstract_elements import Selector
from revolve2.experimentation.optimization.ea import population_management, selection
import numpy as np
import numpy.typing as npt
from typing import Any
from database_components import (
    Individual,
    Population,
)
import config



class ParentSelector(Selector):
    """Selector class for parent selection."""

    rng: np.random.Generator
    offspring_size: int

    def __init__(self, offspring_size: int, rng: np.random.Generator,
                generational=config.GENERATIONAL, steady_state=config.STEADY_STATE,
                selection_func=selection.tournament) -> None:
        """
        Initialize the parent selector.

        :param offspring_size: The offspring size.
        :param rng: The rng generator.
        """
        self.offspring_size = offspring_size
        self.rng = rng
        self.generational = generational
        self.steady_state = steady_state
        self.selection_func = selection_func


    def select(
        self, population: Population, **kwargs: Any
    ) -> tuple[npt.NDArray[np.int_], dict[str, Population]]:
        """
        Select the parents.

        :param population: The population of robots.
        :param kwargs: Other parameters.
        :return: The parent pairs.
        """
        if self.generational:
            return np.array(
                [
                    selection.multiple_unique(
                        selection_size=2,
                        
                        population=[individual.genotype for individual in population.individuals],
                        fitnesses=[individual.fitness for individual in population.individuals],
                        
                        selection_function=lambda _, fitnesses: self.selection_func(
                            rng=self.rng, fitnesses=fitnesses, k=2),
                    )
                    for _ in range(self.offspring_size)
                ],
            ), {"parent_population": population}
        else:
            return np.array(
                [
                    selection.multiple_unique(
                        selection_size=2,
                        
                        population=[individual.genotype for individual in population.individuals],
                        fitnesses=[individual.fitness for individual in population.individuals],
                        
                        selection_function=lambda _, fitnesses: self.selection_func(
                            rng=self.rng, fitnesses=fitnesses, k=2),
                    )
                    for _ in range(self.offspring_size)
                ],
            ), {"parent_population": population}

class SurvivorSelector(Selector):
    """Selector class for survivor selection."""

    rng: np.random.Generator

    def __init__(self, rng: np.random.Generator, generational=config.GENERATIONAL,
                 steady_state=config.STEADY_STATE, selection_func=selection.tournament) -> None:
        """
        Initialize the parent selector.

        :param rng: The rng generator.
        """
        self.rng = rng
        self.generational = generational
        self.steady_state = steady_state
        self.selection_func = selection_func

    def select(
        self, population: Population, **kwargs: Any
    ) -> tuple[Population, dict[str, Any]]:
        """
        Select survivors using a tournament.

        :param population: The population the parents come from.
        :param kwargs: The offspring, with key 'offspring_population'.
        :returns: A newly created population.
        :raises ValueError: If the population is empty.
        """
        offspring = kwargs.get("children")
        offspring_fitness = kwargs.get("child_task_performance")
        if offspring is None or offspring_fitness is None:
            raise ValueError(
                "No offspring was passed with positional argument 'children' and / or 'child_task_performance'."
            )

        if self.generational:
            original_survivors, offspring_survivors = population_management.generational(
                old_genotypes=[i.genotype for i in population.individuals],
                old_fitnesses=[i.fitness for i in population.individuals],
                new_genotypes=offspring,
                new_fitnesses=offspring_fitness,
                selection_function=lambda n, genotypes, fitnesses: selection.multiple_unique(
                    selection_size=n,
                    population=genotypes,
                    fitnesses=fitnesses,
                    selection_function=lambda _, fitnesses: self.selection_func(
                        rng=self.rng, fitnesses=fitnesses, k=2
                    ),
                ),
            )

        else:
            original_survivors, offspring_survivors = population_management.steady_state(
                
                old_genotypes=[i.genotype for i in population.individuals],
                old_fitnesses=[i.fitness for i in population.individuals],
                new_genotypes=offspring,
                new_fitnesses=offspring_fitness,

                selection_function=lambda n, genotypes, fitnesses: selection.multiple_unique(
                    selection_size=n,
                    population=genotypes,
                    fitnesses=fitnesses,
                    selection_function=lambda _, fitnesses: self.selection_func(
                        rng=self.rng, fitnesses=fitnesses, k=2
                    ),
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
                    Individual(genotype=offspring[i],
                               fitness=offspring_fitness[i],
                    )
                    for i in offspring_survivors
                ]
            ),
            {},
        )
