from dataclasses import dataclass
from typing import Generic, TypeVar

from ._measures import Measures
from ._serializable import Serializable

TGenotype = TypeVar("TGenotype", bound=Serializable)
TMeasures = TypeVar("TMeasures", bound=Measures)


@dataclass
class Individual(Generic[TGenotype, TMeasures]):
    """An individual, consisting of a genotype and its measures."""

    genotype: TGenotype
    measures: TMeasures
