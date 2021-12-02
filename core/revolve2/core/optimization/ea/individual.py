from dataclasses import dataclass
from typing import Generic, TypeVar

Genotype = TypeVar("Genotype")
Evaluation = TypeVar("Evaluation")


@dataclass
class Individual(Generic[Genotype, Evaluation]):
    id: int
    genotype: Genotype
    evaluation: Evaluation
