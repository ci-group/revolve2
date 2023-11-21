from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray

from typing import List, TypeVar
import random

from revolve2.experimentation.genotypes.protocols import IGenotype


Genotype = TypeVar("Genotype", bound=IGenotype)


class SelectionAlgorithm(ABC):
    """The bare type of a selection algorithm"""

    @abstractmethod
    def __init__(self, *args, **kwargs) -> None:
        ...

    @abstractmethod
    def __call__(
        self, children: List[Genotype], scores: List[float], result_size: int
    ) -> List[Genotype]:
        ...


class TournamentSelection(SelectionAlgorithm):
    def __init__(self, remove_chosen: bool = False, amount_to_take: int = 2) -> None:
        """A tournament selection algorithm

        ---
        Parameters:
        remove_chosen: bool
            Whether or not to remove previously chosen individuals from the tournament pool
        amount_to_take: int
            The amount of children to take at once for each tournament round
        """
        self.remove_chosen = remove_chosen
        self.amount_to_take = amount_to_take

    def __call__(
        self, children: List[Genotype], scores: List[float], result_size: int
    ) -> List[Genotype]:
        """Tournament selection algorithm

        ---
        Parameters:
        children: NDArray
            Array representing the children to select from
        scores: List[int]
            Array representing the scores of each individual in the population
            Each index should represent the same index in the population
        result_size: int
            The population size of the result

        ---
        Returns:
        NDArray representing the new population
        """
        scores_np = np.asarray(scores)
        output = []
        while len(output) < result_size:
            winner_index = self.tournament_round(scores_np)
            output.append(children[winner_index])
            if self.remove_chosen:
                children.pop(winner_index)
                scores_np = np.delete(scores_np, winner_index, axis=0)
        return output

    def tournament_round(self, scores: NDArray[np.float64]) -> np.int64:
        indexes = np.random.choice(len(scores), size=self.amount_to_take, replace=False)
        return indexes[scores[indexes].argmax()]


class RouletteSelection(SelectionAlgorithm):
    def __init__(self, remove_chosen: bool = False) -> None:
        """A roulette selection algorithm

        ---
        Parameters:
        rate: float [0:1]
            The rate at which to randomly mutate
        """
        self.remove_chosen = remove_chosen

    def __call__(
        self, children: List[Genotype], scores: List[float], result_size: int
    ) -> List[Genotype]:
        """Roulette selection algorithm

        ---
        Parameters:
        children: NDArray
            Array representing the children to select from
        scores: List[int]
            Array representing the scores of each individual in the population
            Each index should represent the same index in the population
        result_size: int
            The population size of the result

        ---
        Returns:
        NDArray representing the new population
        """
        scores_np = np.asarray(scores)
        output = []
        while len(output) < result_size:
            winner_index = self.roulette_wheel(scores_np)
            output.append(children[winner_index])
            if self.remove_chosen:
                children.pop(winner_index)
                scores_np = np.delete(scores_np, winner_index, axis=0)
        return output

    def roulette_wheel(self, scores: NDArray[np.float64]) -> np.int64:
        # Normalisation is to make sure the sum isn't negtive, which ranom.choices doesn't accpet.
        # There is still the 1/m chance of a 0 at first check, tough, which is sloppily dealt with.
        weights = scores / (np.sum(scores) + 1)
        try:
            [value] = random.choices(scores, weights)
        except ValueError:
            value = 0
        return np.where(scores == value)[0][0]


class DeterministicSelection(SelectionAlgorithm):
    def __init__(self) -> None:
        """A deterministic selection algorithm. The best ones always win."""

    def __call__(
        self, children: List[Genotype], scores: List[float], result_size: int
    ) -> List[Genotype]:
        """Roulette selection algorithm

        ---
        Parameters:
        children: NDArray
            Array representing the children to select from
        scores: List[int]
            Array representing the scores of each individual in the population
            Each index should represent the same index in the population
        result_size: int
            The population size of the result

        ---
        Returns:
        NDArray representing the new population
        """
        scores_np = np.asarray(scores)
        output = []
        while len(output) < result_size:
            winner_index = scores_np.argmax()
            output.append(children.pop(winner_index))
            scores_np = np.delete(scores_np, winner_index, axis=0)
        return output
