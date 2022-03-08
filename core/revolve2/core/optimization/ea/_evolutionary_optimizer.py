from typing import Generic, TypeVar

from revolve2.core.optimization import Optimizer
from abc import abstractmethod

Genotype = TypeVar("Genotype")  # TODO bounds
Fitness = TypeVar("Fitness")


class EvolutionaryOptimizer(Optimizer, Generic[Genotype, Fitness]):
    async def ainit(self) -> None:
        pass

    async def ainit_from_database(self) -> bool:
        pass
