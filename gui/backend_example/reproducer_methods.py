
import multineat

import numpy as np
from typing import Any
import numpy.typing as npt
import random
from revolve2.experimentation.evolution.abstract_elements import Reproducer
from database_components import (
    Genotype,
    Population,
)

class CrossoverReproducer(Reproducer):
    """A simple crossover reproducer using multineat."""

    rng: np.random.Generator
    innov_db_body: multineat.InnovationDatabase
    innov_db_brain: multineat.InnovationDatabase

    def __init__(
        self,
        rng: np.random.Generator,
        innov_db_body: multineat.InnovationDatabase,
        innov_db_brain: multineat.InnovationDatabase,
    ):
        """
        Initialize the reproducer.

        :param rng: The ranfom generator.
        :param innov_db_body: The innovation database for the body.
        :param innov_db_brain: The innovation database for the brain.
        """
        self.rng = rng
        self.innov_db_body = innov_db_body
        self.innov_db_brain = innov_db_brain

    # def reproduce(
    #     self, population: npt.NDArray[np.int_], **kwargs: Any
    # ) -> list[Genotype]:
    #     """
    #     Reproduce the population by crossover.

    #     :param population: The parent pairs.
    #     :param kwargs: Additional keyword arguments.
    #     :return: The genotypes of the children.
    #     :raises ValueError: If the parent population is not passed as a kwarg `parent_population`.
    #     """
    #     parent_population: Population | None = kwargs.get("parent_population")
    #     if parent_population is None:
    #         raise ValueError("No parent population given.")

    #     offspring_genotypes = [
    #         Genotype.crossover(
    #             parent_population.individuals[parent1_i].genotype,
    #             parent_population.individuals[parent2_i].genotype,
    #             self.rng,
    #         ).mutate(self.innov_db_body, self.innov_db_brain, self.rng)
    #         for parent1_i, parent2_i in population
    #     ]
    #     return offspring_genotypes
    
    def reproduce(
        self, population: npt.NDArray[np.int_], **kwargs: Any
    ) -> list[Genotype]:
        """
        Reproduce the population by crossover.

        :param population: The parent pairs.
        :param kwargs: Additional keyword arguments.
        :return: The genotypes of the children.
        :raises ValueError: If the parent population is not passed as a kwarg `parent_population`.
        """
        parent_population: Population | None = kwargs.get("parent_population")
        if parent_population is None:
            raise ValueError("No parent population given.")

        offspring_genotypes = []
        while len(offspring_genotypes) <= len(population):

            parent1_i = np.random.randint(0, len(parent_population.individuals))
            parent2_i = random.choice(list(set([x for x in range(0, len(parent_population.individuals))]) - set([parent1_i])))
            # no incest

            offspring_genotypes.append(
                Genotype.crossover(
                    parent_population.individuals[parent1_i].genotype,
                    parent_population.individuals[parent2_i].genotype,
                    self.rng,
                ).mutate(self.innov_db_body, self.innov_db_brain, self.rng)
            )

        return offspring_genotypes