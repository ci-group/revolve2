from __future__ import annotations

import pickle
from dataclasses import dataclass
from typing import Generic, List, Optional, TypeVar, cast

from revolve2.core.database.serialize import Serializable, SerializeError
from revolve2.core.database.view import AnyView

Genotype = TypeVar("Genotype", bound=Serializable)
Fitness = TypeVar("Fitness", bound=Serializable)


@dataclass
class Individual(Generic[Genotype, Fitness], Serializable):
    id: int
    genotype: Genotype
    fitness: Fitness
    parent_ids: Optional[List[int]]  # None means this is from the initial population

    def to_database(self, db_view: AnyView) -> None:
        root = db_view.dict
        root.clear()

        root.insert("id").int = self.id
        root.insert(".genotype_type").bytes = pickle.dumps(type(self.genotype))
        self.genotype.to_database(root.insert("genotype"))

        root.insert(".fitness_type").bytes = pickle.dumps(type(self.fitness))
        fitness = root.insert("fitness")
        # TODO support more types in a more generic way throughout this codebase
        if type(self.fitness) == float:
            fitness.float = cast(float, self.fitness)
        else:
            self.fitness.to_database(fitness)

        parents = root.insert("parents")
        if self.parent_ids is None:
            parents.make_none()
        else:
            parents_list = parents.list
            parents_list.clear()
            for parent in self.parent_ids:
                parents_list.append().int = parent

    @classmethod
    def from_database(cls, db_view: AnyView) -> Individual[Genotype, Fitness]:
        root = db_view.dict

        id = root["id"].int
        genotype_type = pickle.loads(root[".genotype_type"].bytes)
        if not issubclass(genotype_type, Serializable):
            raise SerializeError("Loaded genotype type is not Serializable.")
        genotype = genotype_type.from_database(root["genotype"])

        fitness_type = pickle.loads(root[".fitness_type"].bytes)
        fitness_db = root["fitness"]
        if fitness_type == float:
            fitness = fitness_db.float
        elif issubclass(fitness_type, Serializable):
            fitness = fitness_type.from_database(fitness_db)
        else:
            raise SerializeError("Loaded fitness type is not Serializable.")

        parents_db = root["parents"]
        if parents_db.is_none():
            parents = None
        else:
            parents = [parent.int for parent in parents_db.list]

        return Individual(id, genotype, cast(Fitness, fitness), parents)
