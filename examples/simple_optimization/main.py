from __future__ import annotations

import logging
from dataclasses import dataclass
from random import Random
from typing import List, Optional

import revolve2.core.optimization.ea.population_management as population_management
import revolve2.core.optimization.ea.selection as selection
from revolve2.core.database import Database, Node
from revolve2.core.database.sqlite import Database as DbSqlite
from revolve2.core.optimization.ea import EvolutionaryOptimizer, Individual
from revolve2.serialization import Serializable, StaticData


@dataclass
class Item:
    weight: float
    value: float


class Genotype(Serializable):
    items: List[bool]

    def __init__(self, items: List[bool]) -> None:
        self.items = items

    @classmethod
    def random(cls, rng: Random, has_item_prob: float, num_items: int) -> Genotype:
        return cls([rng.random() < has_item_prob for _ in range(num_items)])

    def develop(self, items: List[Item], maximum_weight: float) -> Phenotype:
        phenotype = []
        total_weight = 0
        for has_item, item in zip(self.items, items):
            if has_item and total_weight + item.weight < maximum_weight:
                phenotype.append(True)
            else:
                phenotype.append(False)

        return Phenotype(phenotype)

    def serialize(self) -> StaticData:
        return self.items

    @classmethod
    def deserialize(cls, data: StaticData) -> Serializable:
        assert isinstance(data, list)
        assert all([isinstance(item, bool) for item in data])
        return cls(data)


class Phenotype(List[bool]):
    items: List[bool]

    def __init__(self, items: List[bool]) -> None:
        self.items = items


class Optimizer(EvolutionaryOptimizer[Genotype, float]):
    _items: List[Item]
    _num_generations: int

    def __init__(self) -> None:
        pass

    @classmethod
    async def create(
        cls,
        database: Database,
        rng: Random,
        population_size: int,
        offspring_size: int,
        initial_population: List[Genotype],
        initial_fitness: Optional[List[float]],
        items: List[Item],
        num_generations: int,
    ) -> Optimizer:
        self = Optimizer()

        await super(Optimizer, self).asyncinit(
            database,
            database.root,
            rng,
            population_size,
            offspring_size,
            initial_population,
            initial_fitness,
        )

        self._items = items
        self._num_generations = num_generations

        return self

    async def _evaluate_generation(
        self, individuals: List[Genotype], database: Database, dbview: Node
    ) -> List[float]:
        return [
            float(
                sum(
                    [
                        has_items * item.value
                        for has_items, item in zip(individual.items, self._items)
                    ]
                )
            )
            for individual in individuals
        ]

    def _select_parents(
        self,
        generation: List[Individual[Genotype, float]],
        num_parents: int,
    ) -> List[List[Individual[Genotype, float]]]:
        return [
            [
                i[0]
                for i in selection.multiple_unique(
                    [(i, i.fitness) for i in generation],
                    2,
                    lambda gen: selection.tournament(self._rng, gen, k=2),
                )
            ]
            for _ in range(num_parents)
        ]

    def _select_survivors(
        self,
        old_individuals: List[Individual[Genotype, float]],
        new_individuals: List[Individual[Genotype, float]],
        num_survivors: int,
    ) -> List[Individual[Genotype, float]]:
        assert len(old_individuals) == num_survivors

        return [
            i[0]
            for i in population_management.steady_state(
                [(i, i.fitness) for i in old_individuals],
                [(i, i.fitness) for i in new_individuals],
                lambda pop: selection.tournament(self._rng, pop, k=2),
            )
        ]

    def _crossover(self, parents: List[Genotype]) -> Genotype:
        assert len(parents) == 2
        point = self._rng.randrange(0, len(parents[0].items))
        return Genotype(parents[0].items[0:point] + parents[1].items[point:])

    def _mutate(self, individual: Genotype) -> Genotype:
        return Genotype(
            [
                has_item ^ (self._rng.random() < 1 / len(individual.items))
                for has_item in individual.items
            ]
        )

    def _must_do_next_gen(self) -> bool:
        return self.generation_index != self._num_generations


async def main() -> None:
    POPULATION_SIZE = 100
    OFFSPRING_SIZE = 100
    NUM_GENERATIONS = 25

    INITIAL_HAS_ITEM_PROB = 0.5

    logging.basicConfig(
        level=logging.DEBUG,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    logging.info(f"Starting optimization")

    # random number generator
    rng = Random()
    rng.seed(100)

    # create 100 random items
    items = [Item(rng.randrange(0, 100), rng.randrange(0, 100)) for _ in range(100)]

    # database
    database = await DbSqlite.create(f"database")

    initial_population = [
        Genotype.random(rng, INITIAL_HAS_ITEM_PROB, len(items))
        for _ in range(POPULATION_SIZE)
    ]

    ep = await Optimizer.create(
        database,
        rng=rng,
        population_size=POPULATION_SIZE,
        offspring_size=OFFSPRING_SIZE,
        initial_population=initial_population,
        initial_fitness=None,
        items=items,
        num_generations=NUM_GENERATIONS,
    )

    logging.info("Starting optimization process..")

    await ep.run()

    logging.info(f"Finished optimizing.")


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
