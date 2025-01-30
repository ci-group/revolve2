import random
from typing import TypeVar

from ._argsort import argsort
from ._supports_lt import SupportsLt

Genotype = TypeVar("Genotype")
Fitness = TypeVar("Fitness", bound=SupportsLt)

def roulette(n: int, genotypes: list[Genotype], fitnesses: list[Fitness]) -> list[int]:
    """
    Perform roulette wheel selection to choose n genotypes probabilistically based on fitness.

    :param n: The number of genotypes to select.
    :param genotypes: The genotypes. Ignored, but kept for compatibility with other selection functions.
    :param fitnesses: Fitnesses of the genotypes.
    :returns: Indices of the selected genotypes.
    """
    assert len(fitnesses) >= n, "Number of selections cannot exceed population size"
    
    # Normalize fitness values to ensure all are positive
    min_fitness = min(fitnesses)
    if min_fitness < 0:
        fitnesses = [f - min_fitness for f in fitnesses]  # Shift all values to be positive
    
    total_fitness = sum(fitnesses)
    assert total_fitness > 0, "Total fitness must be greater than zero for roulette selection"
    
    # Compute selection probabilities
    probabilities = [f / total_fitness for f in fitnesses]
    
    # Perform roulette wheel selection
    selected_indices = random.choices(range(len(fitnesses)), weights=probabilities, k=n)
    
    return selected_indices