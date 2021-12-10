from __future__ import annotations

import pickle
from dataclasses import dataclass
from typing import Generic, List, Optional, TypeVar, cast

from revolve2.core.database import Data
from revolve2.core.database.serialize import Serializable, SerializeError

Genotype = TypeVar("Genotype", bound=Serializable)
Fitness = TypeVar("Fitness", bound=Serializable)


@dataclass
class Individual(Generic[Genotype, Fitness], Serializable):
    id: int
    genotype: Genotype
    fitness: Fitness
    parent_ids: Optional[List[int]]  # None means this is from the initial population

    def serialize(self) -> Data:
        if type(self.fitness) == float:
            fitness_serialized = cast(float, self.fitness)
        else:
            fitness_serialized = self.fitness.serialize()

        return {
            "id": self.id,
            ".genotype_type": pickle.dumps(type(self.genotype)),
            "genotype": self.genotype,
            ".fitness_type": pickle.dumps(type(self.fitness)),
            "fitness": fitness_serialized,
            "parents": self.parent_ids,
        }

    @classmethod
    def deserialize(cls, data: Data) -> Individual[Genotype, Fitness]:
        if type(data) != dict:
            raise SerializeError()

        id_data = data.get("id")
        if id_data is None or type(id_data) != int:
            raise SerializeError()
        id = cast(int, id_data)

        genotype_type_data = data.get(".genotype_type")
        if genotype_type_data is None or type(genotype_type_data) != bytes:
            raise SerializeError()
        genotype_type = pickle.loads(genotype_type_data)  # TODO catch error
        if not issubclass(genotype_type, Serializable):
            raise SerializeError()

        genotype_data = data.get("genotype")
        if genotype_data is None:
            raise SerializeError()
        genotype = cast(Genotype, genotype_type.deserialize(genotype_data))

        fitness_type_data = data.get(".fitness_type")
        if fitness_type_data is None or type(fitness_type_data) != bytes:
            raise SerializeError()
        fitness_type = pickle.loads(fitness_type_data)  # TODO catch error
        if not issubclass(fitness_type, Serializable):
            raise SerializeError()

        fitness_data = data.get("fitness")
        if fitness_data is None:
            raise SerializeError()
        fitness = cast(Fitness, fitness_type.deserialize(fitness_data))

        parents_data = data.get("parents")
        if parents_data is None:
            parent_ids = None
        else:
            if type(parents_data) != list:
                raise SerializeError()
            parent_ids: List[int] = []
            for parent_data in parents_data:
                if type(parent_data) != int:
                    raise SerializeError()
                parent_ids.append(parent_data)

        return Individual(id, genotype, fitness, parent_ids)
