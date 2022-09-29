from revolve2.core.optimization.ea.population import Individual
from revolve2.core.optimization.ea.population.pop_list import (
    PopList,
    multiple_unique,
    tournament,
    topn,
)
from typing import List
import numpy as np


class Genotype:
    pass


TPop = PopList[Genotype]


def measure_displacements(genotypes: List[Genotype]) -> List[float]:
    return [0.0 for _ in genotypes]  # TODO


def measure(pop: TPop) -> None:
    displacements = measure_displacements([i.genotype for i in pop.individuals])
    for individual, displacement in zip(pop.individuals, displacements):
        individual.measures["displacement"] = displacement


def mutate(genotype: Genotype) -> Genotype:
    return Genotype()  # TODO


def crossover(parent1: Genotype, parent2: Genotype) -> Genotype:
    return Genotype()


def evolve(rng: np.random.Generator, pop: TPop) -> TPop:
    OFFSPRING_SIZE = 50
    population_size = len(pop.individuals)

    parent_groups = [
        multiple_unique(pop, 2, lambda pop: tournament(pop, "displacement", rng, k=2))
        for _ in range(OFFSPRING_SIZE)
    ]

    offspring = TPop(
        [
            Individual(
                mutate(
                    crossover(pop.individuals[parents[0]], pop.individuals[parents[1]])
                )
            )
            for parents in parent_groups
        ]
    )
    measure(offspring)

    original_selection, offspring_selection = topn(
        pop, offspring, measure="displacement", n=population_size
    )

    return TPop.from_existing_populations(
        [pop, offspring], [original_selection, offspring_selection], ["displacement"]
    )


def save_state(gen_index: int, pop: TPop):
    pass


rng = np.random.Generator(np.random.PCG64(0))

initial_pop = TPop(
    [
        Individual(Genotype()),
        Individual(Genotype()),
        Individual(Genotype()),
        Individual(Genotype()),
    ]
)
measure(initial_pop)

pop = initial_pop
for _ in range(100):
    pop = evolve(rng, pop)
