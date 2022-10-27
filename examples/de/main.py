"""Optimize a neural network for solving XOR."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from revolve2.core.database import (
    SerializableIncrementableStruct,
    open_async_database_sqlite,
)
from revolve2.core.database.std import Rng
from revolve2.core.optimization.ea.algorithms import de_offspring
from revolve2.core.optimization.ea.population import (
    Individual,
    Parameters,
    SerializableMeasures,
)
from revolve2.core.optimization.ea.population.pop_list import PopList, topn
from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy.ext.asyncio.session import AsyncSession

Genotype = Parameters


@dataclass
class Measures(SerializableMeasures, table_name="measures"):
    """Measures of a genotype/phenotype."""

    result00: Optional[float] = None
    result10: Optional[float] = None
    result01: Optional[float] = None
    result11: Optional[float] = None
    error00: Optional[float] = None
    error10: Optional[float] = None
    error01: Optional[float] = None
    error11: Optional[float] = None
    fitness: Optional[float] = None


class Population(PopList[Genotype, Measures], table_name="population"):
    """A population of individuals consisting of the above Genotype and Measures."""

    pass


@dataclass
class ProgramState(
    SerializableIncrementableStruct,
    table_name="program_state",
):
    """State of the program."""

    rng: Rng
    population: Population
    generation_index: int


class Optimizer:
    """Program that optimizes the neural network parameters."""

    NUM_PARAMS: int = 9
    POPULATION_SIZE: int = 100
    OFFSPRING_SIZE: int = 50
    CROSSOVER_PROBABILITY: float = 0.9
    DIFFERENTIAL_WEIGHT: float = 0.8

    db: AsyncEngine

    state: ProgramState

    async def run(self) -> None:
        """Run the program."""
        self.db = open_async_database_sqlite("database")
        async with self.db.begin() as conn:
            await ProgramState.prepare_db(conn)

        if not await self.load_state():
            rng_seed = 0
            initial_rng = Rng(np.random.Generator(np.random.PCG64(rng_seed)))

            initial_population = Population(
                [
                    Individual(
                        Genotype(
                            [
                                float(v)
                                for v in initial_rng.rng.random(size=self.NUM_PARAMS)
                            ]
                        ),
                        Measures(),
                    )
                    for _ in range(self.POPULATION_SIZE)
                ]
            )
            self.measure(initial_population)

            initial_generation_index = 0

            self.state = ProgramState(
                rng=initial_rng,
                population=initial_population,
                generation_index=initial_generation_index,
            )

            await self.save_state()

        while self.state.generation_index < 300:
            self.evolve()
            await self.save_state()

    async def save_state(self) -> None:
        """Save the state of the program."""
        async with AsyncSession(self.db) as ses:
            async with ses.begin():
                await self.state.to_db(ses)

    async def load_state(self) -> bool:
        """
        Load the state of the program.

        :returns: True if could be loaded from database. False if no data available.
        """
        async with AsyncSession(self.db) as ses:
            async with ses.begin():
                maybe_state = await ProgramState.from_db_latest(ses, 1)
                if maybe_state is None:
                    return False
                else:
                    self.state = maybe_state
                    return True

    def evolve(self) -> None:
        """Iterate one generation further."""
        self.state.generation_index += 1

        offspring = Population(
            [
                Individual(genotype, Measures())
                for genotype in de_offspring(
                    self.state.population,
                    self.state.rng,
                    self.DIFFERENTIAL_WEIGHT,
                    self.CROSSOVER_PROBABILITY,
                )
            ]
        )

        self.measure(offspring)

        original_selection, offspring_selection = topn(
            self.state.population, offspring, measure="fitness", n=self.POPULATION_SIZE
        )

        self.state.population = Population.from_existing_populations(  # type: ignore # TODO
            [self.state.population, offspring],
            [original_selection, offspring_selection],
            [  # TODO make them not copied measures
                "result00",
                "result10",
                "result01",
                "result11",
                "error00",
                "error10",
                "error01",
                "error11",
                "fitness",
            ],
        )

    def measure(self, pop: Population) -> None:
        """
        Measure all individuals in a population.

        :param pop: The population.
        """
        for individual in pop:
            self.measure_one(individual)

    def measure_one(self, individual: Individual[Genotype, Measures]) -> None:
        """
        Measure one individual.

        :param individual: The individual to measure.
        """

        def relu(val: float) -> float:
            return max(0, val)

        def evaluate_network(
            params: List[float], input1: float, input2: float
        ) -> float:
            # usually you would do this with matrix multiplications and numpy,
            # but leaving it manualy for clarity
            n0 = relu(input1 * params[0] + input2 * params[1] + params[2])
            n1 = relu(input1 * params[3] + input2 * params[4] + params[5])
            return relu(n0 * params[6] + n1 * params[7] + params[8])

        ios = [(0, 0, 0), (1, 0, 1), (0, 1, 1), (1, 1, 0)]

        results = [evaluate_network(individual.genotype, io[0], io[1]) for io in ios]
        errors = [abs(result - io[2]) for result, io in zip(results, ios)]

        individual.measures.result00 = results[0]
        individual.measures.result10 = results[1]
        individual.measures.result01 = results[2]
        individual.measures.result11 = results[3]

        individual.measures.error00 = errors[0]
        individual.measures.error10 = errors[1]
        individual.measures.error01 = errors[2]
        individual.measures.error11 = errors[3]

        individual.measures.fitness = sum([-(err**2) for err in errors])


async def main() -> None:
    """Run the program."""
    await Optimizer().run()


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
