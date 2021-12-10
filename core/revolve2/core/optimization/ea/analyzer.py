from typing import Any, Iterator, List, Optional

from multineat import RetrieveGenomeList
from revolve2.core.database import Data, View
from revolve2.core.database.serialize import SerializeError


class IndividualId(int):
    pass


class Generation:
    _view: View
    _gen_num: int

    def __init__(self, view: View, gen_num: int):
        self._view = view.list
        self._gen_num = gen_num

    def __iter__(self) -> Iterator[IndividualId]:
        for id in self.individual_ids:
            yield id

    @property
    def individual_ids(self) -> List[IndividualId]:
        data = self._view.data
        if type(data) != list:
            raise SerializeError()
        ids = []
        for id in data:
            if type(id) != int:
                raise SerializeError()
            ids.append(IndividualId(id))
        return ids

    @property
    def gen_num(self) -> int:
        return self._gen_num


class Generations:
    _view: View

    def __init__(self, view: View):
        self._view = view.list

    def __getitem__(self, index: int) -> Generation:
        return Generation(self._view[index], index)

    def __len__(self) -> int:
        return len(self._view)

    def __iter__(self) -> Iterator[Generation]:
        for i in range(len(self)):
            yield Generation(self._view[i], i)


class Individual:
    _id: int
    _parents: Optional[List[int]]
    _genotype: Data
    _fitness: Data

    def __init__(self, view: View):
        data = view.data
        if type(data) != dict:
            raise SerializeError()

        id_data = data.get("id")
        if id_data is None or type(id_data) != int:
            raise SerializeError()
        self._id = id_data

        parents_data = data.get("parents")
        if parents_data is None:
            self._parents = None
        else:
            if type(parents_data) != list:
                raise SerializeError()
            self._parents = []
            for parent_data in parents_data:
                if type(parent_data) != int:
                    raise SerializeError()
                self._parents.append(parent_data)

        genotype_data = data.get("genotype")
        if genotype_data is None:
            raise SerializeError()
        self._genotype = genotype_data

        fitness_data = data.get("fitness")
        if fitness_data is None:
            raise SerializeError()
        self._fitness = fitness_data

    @property
    def id(self) -> IndividualId:
        return self._id

    @property
    def parents(self) -> Optional[List[IndividualId]]:
        return self._parents

    @property
    def genotype(self) -> Data:
        return self._genotype

    @property
    def fitness(self) -> Data:
        return self._fitness


class Individuals:
    _view: View

    def __init__(self, view: View):
        self._view = view.list

    def __getitem__(self, index: int) -> Individual:
        return Individual(self._view[index])


class Evaluations:
    pass


"""
    _view: View

    def __init__(self, view: AnyView):
        self._view = view.list

    def __getitem__(self, index: int) -> AnyView:
        return self._view[index]

    def __len__(self) -> int:
        return len(self._view)

    def __iter__(self) -> Iterator[AnyView]:
        for i in range(len(self)):
            yield self._view[i]
"""


class Analyzer:
    _database: View

    def __init__(self, database: View):
        self._database = database

    @property
    def generations(self) -> Generations:
        return Generations(self._database["generations"])

    @property
    def individuals(self) -> Individuals:
        return Individuals(self._database["individuals"])

    @property
    def offspring_size(self) -> int:
        return self._database["offspring_size"].int

    @property
    def population_size(self) -> int:
        return self._database["population_size"].int

    @property
    def evaluations(self) -> Evaluations:
        # return Evaluations(self._database["evaluations"])
        raise NotImplementedError()
