"""Individual class."""

from dataclasses import dataclass

from .genotype import Genotype


@dataclass
class Individual:
    """An individual in a population."""

    genotype: Genotype
    fitness: float
