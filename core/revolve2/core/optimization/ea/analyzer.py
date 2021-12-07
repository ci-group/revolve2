from typing import Any, Iterator, List, Optional

from revolve2.core.database.view import AnyView, ListView
from revolve2.core.database.view.dict_view import DictView


class IndividualId(int):
    pass


class Generation:
    _view: ListView
    _gen_num: int

    def __init__(self, view: AnyView, gen_num: int):
        self._view = view.list
        self._gen_num = gen_num

    def __getitem__(self, index: int) -> IndividualId:
        return IndividualId(self._view[index].int)

    def __len__(self) -> int:
        return len(self._view)

    def __iter__(self) -> Iterator[IndividualId]:
        for i in range(len(self)):
            yield IndividualId(self._view[i].int)

    @property
    def gen_num(self) -> int:
        return self._gen_num


class Generations:
    _view: ListView

    def __init__(self, view: AnyView):
        self._view = view.list

    def __getitem__(self, index: int) -> Generation:
        return Generation(self._view[index], index)

    def __len__(self) -> int:
        return len(self._view)

    def __iter__(self) -> Iterator[Generation]:
        for i in range(len(self)):
            yield Generation(self._view[i], i)


class Individual:
    _view: DictView

    def __init__(self, view: AnyView):
        self._view = view.dict

    @property
    def id(self) -> IndividualId:
        return self._view["id"].int

    @property
    def parents(self) -> Optional[List[IndividualId]]:
        parents_db = self._view["parents"]
        if parents_db.is_none():
            return None
        else:
            return [IndividualId(p.int) for p in parents_db.list]

    @property
    def genotype(self) -> AnyView:
        return self._view["genotype"]

    @property
    def fitness(self) -> AnyView:
        return self._view["fitness"]


class Individuals:
    _view: ListView

    def __init__(self, view: AnyView):
        self._view = view.list

    def __getitem__(self, index: int) -> Individual:
        return Individual(self._view[index])

    def __len__(self) -> int:
        return len(self._view)

    def __iter__(self) -> Iterator[Individual]:
        for i in range(len(self)):
            yield Individual(self._view[i])


class Evaluations:
    _view: ListView

    def __init__(self, view: AnyView):
        self._view = view.list

    def __getitem__(self, index: int) -> AnyView:
        return self._view[index]

    def __len__(self) -> int:
        return len(self._view)

    def __iter__(self) -> Iterator[AnyView]:
        for i in range(len(self)):
            yield self._view[i]


class Analyzer:
    _database: DictView

    def __init__(self, database: AnyView):
        self._database = database.dict

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
        return Evaluations(self._database["evaluations"])
