from __future__ import annotations

import pickle
from dataclasses import dataclass
from typing import Generic, TypeVar

from revolve2.core.database.view import AnyView

Genotype = TypeVar("Genotype")
Evaluation = TypeVar("Evaluation")


@dataclass
class Individual(Generic[Genotype, Evaluation]):
    id: int
    genotype: Genotype
    evaluation: Evaluation

    def to_database(self, db_view: AnyView) -> None:
        root = db_view.dict
        root.clear()

        root.insert("id").int = self.id
        root.insert("genotype").bytes = pickle.dumps(self.genotype)
        root.insert("evaluation").bytes = pickle.dumps(self.evaluation)

    @classmethod
    def from_database(cls, db_view: AnyView) -> Individual[Genotype, Evaluation]:
        root = db_view.dict

        id = root["id"].int
        genotype = pickle.loads(root["genotype"].bytes)
        evaluation = pickle.loads(root["evaluation"].bytes)

        return cls(id, genotype, evaluation)
