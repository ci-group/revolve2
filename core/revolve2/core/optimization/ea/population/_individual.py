from dataclasses import dataclass
from typing import Generic, TypeVar

TGenotype = TypeVar("TGenotype")
TMeasures = TypeVar("TMeasures")


@dataclass
class Individual(Generic[TGenotype, TMeasures]):
    genotype: TGenotype
    measures: TMeasures
