from __future__ import annotations

from random import Random
from typing import List, Tuple

import revolve2.core.optimization.ea.population_management as population_management
import revolve2.core.optimization.ea.selection as selection
from revolve2.core.database import Database
from revolve2.core.optimization.ea import EvolutionaryOptimizer
from genotype import Genotype
from fitness import Fitness
from item import Item
from revolve2.core.optimization import ProcessIdGen


class Optimizer(EvolutionaryOptimizer["Optimizer", Genotype, Fitness]):
    _rng: Random
    _items: List[Item]
    _num_generations: int

    async def ainit_new(
        self,
        database: Database,
        process_id: int,
        process_id_gen: ProcessIdGen,
        offspring_size: int,
        initial_population: List[Genotype],
        rng: Random,
        items: List[Item],
        num_generations: int,
    ) -> None:
        await super().ainit_new(
            database=database,
            process_id=process_id,
            process_id_gen=process_id_gen,
            genotype_type=Genotype,
            fitness_type=Fitness,
            offspring_size=offspring_size,
            initial_population=initial_population,
        )

        self._rng = rng
        self._items = items
        self._num_generations = num_generations

        # TODO database things

    async def ainit_from_database(
        self,
        database: Database,
        process_id: int,
        process_id_gen: ProcessIdGen,
        rng: Random,
    ) -> bool:
        if not await super().ainit_from_database(
            database=database,
            process_id=process_id,
            process_id_gen=process_id_gen,
            genotype_type=Genotype,
            fitness_type=Fitness,
        ):
            return False

        raise NotImplementedError()

    async def _evaluate_generation(
        self,
        genotypes: List[Genotype],
        database: Database,
        process_id: int,
        process_id_gen: ProcessIdGen,
    ) -> List[Fitness]:
        return [
            Fitness(
                sum(
                    [
                        has_items * item.value
                        for has_items, item in zip(genotype.items, self._items)
                    ]
                )
            )
            for genotype in genotypes
        ]

    def _select_parents(
        self,
        population: List[Genotype],
        fitnesses: List[Fitness],
        num_parent_groups: int,
    ) -> List[List[int]]:
        return [
            selection.multiple_unique(
                population,
                fitnesses,
                2,
                lambda _, fitnesses: selection.tournament(self._rng, fitnesses, k=2),
            )
            for _ in range(num_parent_groups)
        ]

    def _select_survivors(
        self,
        old_individuals: List[Genotype],
        old_fitnesses: List[Fitness],
        new_individuals: List[Genotype],
        new_fitnesses: List[Fitness],
        num_survivors: int,
    ) -> Tuple[List[int], List[int]]:
        assert len(old_individuals) == num_survivors

        return population_management.steady_state(
            old_individuals,
            old_fitnesses,
            new_individuals,
            new_fitnesses,
            lambda _, fitnesses: selection.tournament(self._rng, fitnesses, k=2),
        )

    def _crossover(self, parents: List[Genotype]) -> Genotype:
        assert len(parents) == 2
        point = self._rng.randrange(0, len(parents[0].items))
        return Genotype(parents[0].items[0:point] + parents[1].items[point:])

    def _mutate(self, genotype: Genotype) -> Genotype:
        return Genotype(
            [
                has_item ^ (self._rng.random() < 1 / len(genotype.items))
                for has_item in genotype.items
            ]
        )

    def _must_do_next_gen(self) -> bool:
        return self.generation_index != self._num_generations
