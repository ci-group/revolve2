from __future__ import annotations

from dataclasses import dataclass
from .._individual import Individual
from typing import List, TypeVar, Generic

TGenotype = TypeVar("TGenotype")


@dataclass
class PopList(Generic[TGenotype]):
    individuals: List[Individual[TGenotype]]

    def from_existing_populations(
        populations: List[PopList[TGenotype]],
        selections: List[List[int]],
        copied_measures: List[str],
    ) -> PopList[TGenotype]:
        new_individuals: List[Individual[TGenotype]] = []
        for pop, selection in zip(populations, selections):
            for i in selection:
                new_ind = Individual(pop.individuals[i].genotype)
                for measure in copied_measures:
                    new_ind.measures[measure] = pop.individuals[i].measures[measure]
                new_individuals.append(new_ind)

        return PopList(new_individuals)
