from __future__ import annotations

import pickle
from dataclasses import dataclass
from typing import Generic, List, Optional, TypeVar, Union

from revolve2.core.database import StaticData
from revolve2.core.database.serialization import (
    Serializable,
    SerializeError,
    deserialize,
    serialize,
)

Genotype = TypeVar("Genotype", bound=Union[Serializable, StaticData])
Fitness = TypeVar("Fitness", bound=Union[Serializable, StaticData])


@dataclass
class Individual(Generic[Genotype, Fitness], Serializable):
    id: int
    genotype: Genotype
    fitness: Fitness
    parent_ids: Optional[List[int]]  # None means this is from the initial population

    def serialize(self) -> StaticData:
        return {
            "id": self.id,
            ".genotype_type": pickle.dumps(type(self.genotype)),
            "genotype": serialize(self.genotype),
            ".fitness_type": pickle.dumps(type(self.fitness)),
            "fitness": serialize(self.fitness),
            "parents": self.parent_ids,
        }

    @classmethod
    def deserialize(cls, data: StaticData) -> Individual[Genotype, Fitness]:
        if not isinstance(data, dict):
            raise SerializeError()

        id = data.get("id")
        if not isinstance(id, int):
            raise SerializeError()

        genotype_type_data = data.get(".genotype_type")
        if genotype_type_data is None or type(genotype_type_data) != bytes:
            raise SerializeError()
        genotype_type = pickle.loads(genotype_type_data)  # TODO catch error
        if "genotype" not in data:
            raise SerializeError()
        genotype_data = data.get("genotype")
        genotype = deserialize(genotype_data, genotype_type)

        fitness_type_data = data.get(".fitness_type")
        if fitness_type_data is None or type(fitness_type_data) != bytes:
            raise SerializeError()
        fitness_type = pickle.loads(fitness_type_data)  # TODO catch error
        if "fitness" not in data:
            raise SerializeError()
        fitness_data = data.get("fitness")
        fitness = deserialize(fitness_data, fitness_type)

        parent_ids: Optional[List[int]]
        parents_data = data.get("parents")
        if parents_data is None:
            parent_ids = None
        else:
            if type(parents_data) != list:
                raise SerializeError()
            parent_ids = []
            for parent_data in parents_data:
                if not isinstance(parent_data, int):
                    raise SerializeError()
                parent_ids.append(parent_data)

        return Individual(id, genotype, fitness, parent_ids)
