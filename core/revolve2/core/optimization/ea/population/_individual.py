from dataclasses import dataclass, field
from typing import TypeVar, Generic
from ._measures import Measures

TGenotype = TypeVar("TGenotype")


@dataclass
class Individual(Generic[TGenotype]):
    genotype: TGenotype
    measures: Measures = field(default_factory=dict, init=False)
