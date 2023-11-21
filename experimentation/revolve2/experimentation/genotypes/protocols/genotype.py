from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Self

from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from revolve2.modular_robot import Body
    import numpy as np


class GenotypeInitParams(ABC):
    @abstractmethod
    def __init__(self) -> None:
        ...


class IGenotype(ABC):
    @abstractmethod
    def __init__(self, params: GenotypeInitParams) -> None:
        ...

    @abstractmethod
    def develop(self) -> Body:
        """Develop the genotype into its phenotype"""

    @abstractmethod
    def copy(self) -> Self:
        """Get a deeply copied version of the object"""

    @abstractmethod
    def mutate(self, rng: np.random.Generator) -> Self:
        """Get a deeply copied version of the object, with some mutation applied"""

    @abstractmethod
    def crossover(self, rng: np.random.Generator, *__o: Self) -> Self:
        """Perform crossover between two individuals. Return a new copy"""

    @classmethod
    @abstractmethod
    def random(cls, params: GenotypeInitParams, rng: np.random.Generator) -> Self:
        """Factory method returning a randomly generated individual"""

    @classmethod
    def random_individuals(
        cls, params: GenotypeInitParams, n: int, rng: np.random.Generator
    ) -> List[Self]:
        """Factory method returning randomly generated individuals"""
        return [cls.random(params, rng) for _ in range(n)]
