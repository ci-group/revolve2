from revolve2.core.optimization.ea.population import Individual
from revolve2.core.optimization.ea.population.pop_list import (
    PopList,
    multiple_unique,
    tournament,
    topn,
)
from typing import List, Optional
import numpy as np
from dataclasses import dataclass


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


@dataclass
class State:
    rng: np.random.Generator
    pop: TPop


def save_state(state: State):
    pass


def load_state() -> Optional[State]:
    pass


state = load_state()
if state is None:
    state = State(
        rng=np.random.Generator(np.random.PCG64(0)),
        pop=TPop(
            [
                Individual(Genotype()),
                Individual(Genotype()),
                Individual(Genotype()),
                Individual(Genotype()),
            ]
        ),
    )

measure(state.pop)

save_state(state)

for _ in range(100):
    state.pop = evolve(state.rng, state.pop)
    save_state(state)
