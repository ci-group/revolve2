from dataclasses import dataclass
from typing import Generic, TypeVar

from ._db_serializable import DbSerializable
from ._measures import Measures

TGenotype = TypeVar("TGenotype", bound=DbSerializable)
TMeasures = TypeVar("TMeasures", bound=Measures)


@dataclass
class Individual(Generic[TGenotype, TMeasures]):
    """An individual, consisting of a genotype and its measures."""

    genotype: TGenotype
    measures: TMeasures
