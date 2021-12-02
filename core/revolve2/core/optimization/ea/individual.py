from __future__ import annotations

import pickle
from dataclasses import dataclass
from typing import Generic, TypeVar

from revolve2.core.database.serialize import Serializable, SerializeError
from revolve2.core.database.view import AnyView

Genotype = TypeVar("Genotype", bound=Serializable)
Evaluation = TypeVar("Evaluation", bound=Serializable)


@dataclass
class Individual(Generic[Genotype, Evaluation], Serializable):
    id: int
    genotype: Genotype
    evaluation: Evaluation

    def to_database(self, db_view: AnyView) -> None:
        root = db_view.dict
        root.clear()

        root.insert("id").int = self.id
        root.insert(".genotype_type").bytes = pickle.dumps(type(self.genotype))
        self.genotype.to_database(root.insert("genotype"))

        root.insert(".evaluation_type").bytes = pickle.dumps(type(self.evaluation))
        evaluation = root.insert("evaluation")
        # TODO support more types in a more generic way throughout this codebase
        if type(self.evaluation) == float:
            evaluation.float = self.evaluation
        else:
            self.evaluation.to_database(evaluation)

    @classmethod
    def from_database(cls, db_view: AnyView) -> Individual[Genotype, Evaluation]:
        root = db_view.dict

        id = root["id"].int
        genotype_type = pickle.loads(root[".genotype_type"].bytes)
        if not issubclass(genotype_type, Serializable):
            raise SerializeError("Loaded genotype type is not Serializable.")
        genotype = genotype_type.from_database(root["genotype"])

        evaluation_type = pickle.loads(root[".evaluation_type"].bytes)
        evaluation = root["evaluation"]
        if evaluation_type == float:
            evaluation = evaluation.float
        elif issubclass(evaluation_type, Serializable):
            evaluation = evaluation_type.from_database(evaluation)
        else:
            raise SerializeError("Loaded evaluation type is not Serializable.")

        return Individual(id, genotype, evaluation)
