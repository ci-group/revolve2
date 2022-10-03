from dataclasses import dataclass
from typing import TypeVar, Generic

TGenotype = TypeVar("TGenotype")
TMeasures = TypeVar("TMeasures")


@dataclass
class Individual(Generic[TGenotype, TMeasures]):
    genotype: TGenotype
    measures: TMeasures
