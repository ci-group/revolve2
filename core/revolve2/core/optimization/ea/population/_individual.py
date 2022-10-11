from dataclasses import dataclass
from typing import Generic, TypeVar

from revolve2.core.database import Serializable

from ._serializable_measures import SerializableMeasures

TGenotype = TypeVar("TGenotype", bound=Serializable)
TMeasures = TypeVar("TMeasures", bound=SerializableMeasures)


@dataclass
class Individual(Generic[TGenotype, TMeasures]):
    """An individual in a population."""

    genotype: TGenotype
    measures: TMeasures
