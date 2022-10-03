from dataclasses import dataclass, field
from typing import TypeVar, Generic
from ._measures import Measures

TGenotype = TypeVar("TGenotype")
TMeasures = TypeVar("TMeasures")


@dataclass
class Individual(Generic[TGenotype, TMeasures]):
    genotype: TGenotype
    measures: TMeasures
