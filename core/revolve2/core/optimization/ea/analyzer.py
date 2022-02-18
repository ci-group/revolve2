from __future__ import annotations
from typing import Dict, Iterator, List, Optional

from revolve2.core.database import List as DbList
from revolve2.core.database import Node, DbData, Transaction
from revolve2.serialization import StaticData, is_static_data, SerializeError
from revolve2.core.database import DatabaseError


class Generation:
    _txn: Transaction
    _individual_ids: List[int]
    _gen_num: int

    def __init__(self, txn: Transaction, individual_ids: List[int], gen_num: int):
        self._txn = txn
        self._individual_ids = individual_ids
        self._gen_num = gen_num

    def __iter__(self) -> Iterator[int]:
        for id in self._individual_ids:
            yield id

    @property
    def individual_ids(self) -> List[int]:
        return self._individual_ids

    @property
    def gen_num(self) -> int:
        return self._gen_num


class Generations:
    _txn: Transaction
    _list: DbList

    def __init__(self, txn: Transaction, list: DbList):
        self._txn = txn
        self._list = list

    def __getitem__(self, index: int) -> Generation:
        try:
            individual_ids = self._list.get(self._txn, index).get_db_data(self._txn)
            if not isinstance(individual_ids, list) or not all(
                [isinstance(id, int) for id in individual_ids]
            ):
                raise SerializeError()
            return Generation(self._txn, individual_ids, index)
        except DatabaseError as err:
            raise SerializeError() from err

    def __len__(self) -> int:
        return self._list.len(self._txn)

    def __iter__(self) -> Iterator[Generation]:
        for i in range(len(self)):
            yield self[i]


class Individual:
    _id: int
    _parents: Optional[List[int]]
    _genotype: StaticData
    _fitness: StaticData

    def __init__(self, data: Dict[str, DbData]):
        id_data = data.get("id")
        if not isinstance(id_data, int):
            raise SerializeError()
        self._id = id_data

        if "parents" not in data:
            raise SerializeError()
        parents_data = data["parents"]
        if parents_data is not None:
            if not isinstance(parents_data, list) or not all(
                [isinstance(parent, int) for parent in parents_data]
            ):
                raise SerializeError()
        self._parents = parents_data

        if "genotype" not in data:
            raise SerializeError()
        genotype_data = data["genotype"]
        if not is_static_data(genotype_data):
            raise SerializeError()
        self._genotype = genotype_data

        if "fitness" not in data:
            raise SerializeError()
        fitness_data = data["fitness"]
        if not is_static_data(fitness_data):
            raise SerializeError()
        self._fitness = fitness_data

    @property
    def id(self) -> int:
        return self._id

    @property
    def parents(self) -> Optional[List[int]]:
        return self._parents

    @property
    def genotype(self) -> StaticData:
        return self._genotype

    @property
    def fitness(self) -> StaticData:
        return self._fitness


class Individuals:
    _txn: Transaction
    _individuals: DbList

    def __init__(self, txn: Transaction, individuals: DbList):
        self._txn = txn
        self._individuals = individuals

    def __getitem__(self, index: int) -> Individual:
        try:
            individual = self._individuals.get(self._txn, index).get_db_data(self._txn)
            if not isinstance(individual, dict):
                raise SerializeError()
            return Individual(individual)
        except DatabaseError as err:
            raise SerializeError from err

    def __len__(self) -> int:
        return self._individuals.len(self._txn)

    def __iter__(self) -> Iterator[Individual]:
        for i in range(len(self)):
            yield self[i]


class Evaluations:
    _txn: Transaction
    _evals: DbList

    def __init__(self, txn: Transaction, evals: DbList):
        self._txn = txn
        self._evals = evals

    def __getitem__(self, index: int) -> Node:
        try:
            return self._evals.get(self._txn, index)
        except DatabaseError as err:
            raise SerializeError from err

    def __len__(self) -> int:
        return self._evals.len(self._txn)

    def __iter__(self) -> Iterator[Node]:
        for i in range(len(self)):
            yield self[i]


class Analyzer:
    _txn: Transaction
    _ea: Dict[str, DbData]
    _evaluations: DbList

    def __init__(self, txn: Transaction, node: Node):
        self._txn = txn
        root = node.get_db_data(txn)
        if not isinstance(root, Dict) or "ea" not in root or "evaluations" not in root:
            raise SerializeError()
        self._ea = root["ea"].get_db_data(txn)
        if not isinstance(self._ea, dict):
            raise SerializeError()
        self._evaluations = root["evaluations"].get_db_data(txn)
        if not isinstance(self._evaluations, DbList):
            raise SerializeError()

    @property
    def generations(self) -> Generations:
        generations_node = self._ea.get("generations")
        if generations_node is None or not isinstance(generations_node, Node):
            raise SerializeError()
        generations = generations_node.get_db_data(self._txn)
        if not isinstance(generations, DbList):
            raise SerializeError()

        return Generations(self._txn, generations)

    @property
    def individuals(self) -> Individuals:
        individuals_node = self._ea.get("individuals")
        if individuals_node is None or not isinstance(individuals_node, Node):
            raise SerializeError()
        individuals = individuals_node.get_db_data(self._txn)
        if not isinstance(individuals, DbList):
            raise SerializeError()

        return Individuals(self._txn, individuals)

    @property
    def offspring_size(self) -> int:
        offspring_size = self._ea.get("offspring_size")
        if offspring_size is None or not isinstance(offspring_size, int):
            raise SerializeError()
        return offspring_size

    @property
    def population_size(self) -> int:
        population_size = self._ea.get("population_size")
        if population_size is None or not isinstance(population_size, int):
            raise SerializeError()
        return population_size

    @property
    def evaluations(self) -> Evaluations:
        return Evaluations(self._txn, self._evaluations)
