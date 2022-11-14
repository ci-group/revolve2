"""Optimize a neural network for solving XOR."""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np
import revolve2.standard_resources.modular_robots as standard_robots
from pyrr import Quaternion, Vector3
from revolve2.actor_controllers.cpg import CpgNetworkStructure
from revolve2.core.database import (
    SerializableIncrementableStruct,
    open_async_database_sqlite,
)
from revolve2.core.database.std import Rng
from revolve2.core.modular_robot import Body, ModularRobot
from revolve2.core.modular_robot.brains import (
    BrainCpgNetworkStatic,
    make_cpg_network_structure_neighbour,
)
from revolve2.core.optimization.ea.algorithms import bounce_parameters, de_offspring
from revolve2.core.optimization.ea.population import (
    Individual,
    Parameters,
    SerializableMeasures,
)
from revolve2.core.optimization.ea.population.pop_list import PopList, replace_if_better
from revolve2.core.physics.environment_actor_controller import (
    EnvironmentActorController,
)
from revolve2.core.physics.running import ActorState, Batch
from revolve2.core.physics.running import Environment as PhysicsEnv
from revolve2.core.physics.running import PosedActor, Runner
from revolve2.runners.mujoco import LocalRunner
from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy.ext.asyncio.session import AsyncSession

Genotype = Parameters


@dataclass
class Measures(SerializableMeasures, table_name="measures"):
    """Measures of a genotype/phenotype."""

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


class DERobotBrainOptimizer:
    """Program that optimizes the neural network parameters."""

    NUM_GENERATIONS: int = 100
    POPULATION_SIZE: int = 100
    CROSSOVER_PROBABILITY: float = 0.9
    DIFFERENTIAL_WEIGHT: float = 0.2

    SIMULATION_TIME = 30
    SAMPLING_FREQUENCY = 5
    CONTROL_FREQUENCY = 60

    BODY: Body = standard_robots.gecko()
    CPG_NETWORK_STRUCTURE: CpgNetworkStructure = make_cpg_network_structure_neighbour(
        BODY.find_active_hinges()
    )

    db: AsyncEngine

    state: ProgramState

    runner: Runner

    async def run(
        self,
        rng: Rng,
        database: AsyncEngine,
    ) -> None:
        """
        Run the program.

        :param rng: Random number generator for this run.
        :param database: Database to store (intermediate) results in.
        """
        self.db = database

        self._runner = LocalRunner(headless=True, num_simulators=4)

        async with self.db.begin() as conn:
            await ProgramState.prepare_db(conn)

        if not await self.load_state():
            initial_population = Population(
                [
                    Individual(
                        Genotype(
                            [
                                float(v)
                                for v in rng.rng.random(
                                    size=self.CPG_NETWORK_STRUCTURE.num_connections
                                )
                            ]
                        ),
                        Measures(),
                    )
                    for _ in range(self.POPULATION_SIZE)
                ]
            )
            await self.measure(initial_population)

            initial_generation_index = 0

            self.state = ProgramState(
                rng=rng,
                population=initial_population,
                generation_index=initial_generation_index,
            )

            await self.save_state()

        while self.state.generation_index < self.NUM_GENERATIONS:
            await self.evolve()
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

    async def evolve(self) -> None:
        """Iterate one generation further."""
        self.state.generation_index += 1

        offspring = Population(
            [
                Individual(bounce_parameters(genotype), Measures())
                for genotype in de_offspring(
                    self.state.population,
                    self.state.rng,
                    self.DIFFERENTIAL_WEIGHT,
                    self.CROSSOVER_PROBABILITY,
                )
            ]
        )

        await self.measure(offspring)

        original_selection, offspring_selection = replace_if_better(
            self.state.population, offspring, measure="fitness"
        )

        self.state.population = Population.from_existing_populations(  # type: ignore # TODO
            [self.state.population, offspring],
            [original_selection, offspring_selection],
            [
                "fitness",
            ],
        )

    async def measure(self, population: Population) -> None:
        """
        Measure all individuals in a population.

        :param population: The population.
        """
        batch = Batch(
            simulation_time=self.SIMULATION_TIME,
            sampling_frequency=self.SAMPLING_FREQUENCY,
            control_frequency=self.CONTROL_FREQUENCY,
        )

        for individual in population:
            weight_matrix = (
                self.CPG_NETWORK_STRUCTURE.make_connection_weights_matrix_from_params(
                    np.clip(individual.genotype, 0.0, 1.0) * 4.0 - 2.0
                )
            )
            initial_state = self.CPG_NETWORK_STRUCTURE.make_uniform_state(
                math.sqrt(2) / 2.0
            )
            dof_ranges = self.CPG_NETWORK_STRUCTURE.make_uniform_dof_ranges(1.0)

            brain = BrainCpgNetworkStatic(
                initial_state,
                self.CPG_NETWORK_STRUCTURE.num_cpgs,
                weight_matrix,
                dof_ranges,
            )
            actor, controller = ModularRobot(
                self.BODY, brain
            ).make_actor_and_controller()
            bounding_box = actor.calc_aabb()
            env = PhysicsEnv(EnvironmentActorController(controller))
            env.actors.append(
                PosedActor(
                    actor,
                    Vector3(
                        [
                            0.0,
                            0.0,
                            bounding_box.size.z / 2.0 - bounding_box.offset.z,
                        ]
                    ),
                    Quaternion(),
                    [0.0 for _ in controller.get_dof_targets()],
                )
            )
            batch.environments.append(env)

        batch_results = await self._runner.run_batch(batch)

        fitnesses = [
            self.calculate_fitness(
                environment_result.environment_states[0].actor_states[0],
                environment_result.environment_states[-1].actor_states[0],
            )
            for environment_result in batch_results.environment_results
        ]

        for individual, fitness in zip(population, fitnesses):
            individual.measures.fitness = float(fitness)

    @staticmethod
    def calculate_fitness(begin_state: ActorState, end_state: ActorState) -> float:
        """
        Calculate the fitness corresponding to a simulation result.

        :param begin_state: Initial state of the robot. (begin of simulation)
        :param end_state: Final state of the robot. (end of simulation)
        :returns: The calculated fitness. Euclidian distance between initial and final position.
        """
        # distance traveled on the xy plane
        return math.sqrt(
            (begin_state.position[0] - end_state.position[0]) ** 2
            + ((begin_state.position[1] - end_state.position[1]) ** 2)
        )


async def main() -> None:
    """Run the optimization process."""
    RNG_SEED = 0
    DATABASE_NAME = "robot_brain_de_db"

    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    rng = Rng(np.random.Generator(np.random.PCG64(RNG_SEED)))
    database = open_async_database_sqlite(DATABASE_NAME, create=True)

    await DERobotBrainOptimizer().run(
        rng=rng,
        database=database,
    )


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
