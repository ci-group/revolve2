from __future__ import annotations

from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.asyncio import AsyncConnection
from typing import Optional


class DEOptimizer:
    """Differential evolution optimizer for float vector."""

    db: AsyncEngine
    rng: SerializableRng
    pop: Population
    gen_index: int

    @classmethod
    async def prepare_db(cls, conn: AsyncConnection) -> None:
        pass

    @classmethod
    async def from_db(
        cls: DEOptimizer, ses: AsyncSession, id: int
    ) -> Optional[DEOptimizer]:
        pass

    @classmethod
    async def new(
        cls: DEOptimizer, ses: AsyncSession, id: int
    ) -> Optional[DEOptimizer]:
        pass

    async def run(self) -> None:
        """Run the program."""
        self.db = open_async_database_sqlite("database")
        async with self.db.begin() as conn:
            await Population.prepare_db(conn)
            await SerializableRng.prepare_db(conn)
            await conn.run_sync(DbBase.metadata.create_all)

        if not await self.load_state():
            rng_seed = 0
            self.rng = SerializableRng(np.random.Generator(np.random.PCG64(rng_seed)))
            self.pop = Population(
                [
                    Individual(
                        Genotype(
                            params=ParamList(
                                [
                                    float(v)
                                    for v in self.rng.rng.random(size=self.NUM_PARAMS)
                                ]
                            )
                        ),
                        Measures(),
                    )
                    for _ in range(self.POPULATION_SIZE)
                ]
            )
            self.gen_index = 0
            self.measure(self.pop)

            await self.save_state()

        while self.gen_index < 300:
            self.evolve()
            await self.save_state()

    async def save_state(self) -> None:
        """Save the state of the program."""
        async with AsyncSession(self.db) as ses:
            async with ses.begin():
                popid = await self.pop.to_db(ses)

                dbstate = DbState(
                    generation_index=self.gen_index,
                    rng_id=await self.rng.to_db(ses),
                    pop_id=popid,
                )
                ses.add(dbstate)

    async def load_state(self) -> bool:
        """
        Load the state of the program.

        :returns: True if could be loaded from database. False if no data available.
        """
        async with AsyncSession(self.db) as ses:
            async with ses.begin():
                state = (
                    await ses.execute(
                        select(DbState)
                        .order_by(DbState.generation_index.desc())
                        .limit(1)
                    )
                ).scalar_one_or_none()

                if state is None:
                    return False

                self.gen_index = state.generation_index
                maybe_rng = await SerializableRng.from_db(ses, state.rng_id)
                assert maybe_rng is not None
                self.rng = maybe_rng

                maybe_pop = await Population.from_db(ses, state.pop_id)
                assert maybe_pop is not None
                self.pop = maybe_pop  # type: ignore # TODO

                return True

    def evolve(self) -> None:
        """Iterate one generation further."""
        self.gen_index += 1

        offspring_inds: List[Individual] = []

        for individual_i, individual in enumerate(self.pop):
            sub_pop = self.pop[individual_i:]
            selection = self.rng.rng.choice(range(len(sub_pop)), 3)

            x = np.array(individual.genotype.params)
            a = np.array(sub_pop[selection[0]].genotype.params)
            b = np.array(sub_pop[selection[1]].genotype.params)
            c = np.array(sub_pop[selection[2]].genotype.params)

            y = a + self.DIFFERENTIAL_WEIGHT * (b - c)

            crossover_mask = self.rng.rng.binomial(
                n=1, p=self.CROSSOVER_PROBABILITY, size=self.NUM_PARAMS
            )

            child = crossover_mask * x + (1.0 - crossover_mask) * y

            offspring_inds.append(
                Individual(Genotype(ParamList([float(v) for v in child])), Measures())
            )

        offspring = Population(offspring_inds)
        self.measure(offspring)

        original_selection, offspring_selection = topn(
            self.pop, offspring, measure="fitness", n=self.POPULATION_SIZE
        )

        self.pop = Population.from_existing_populations(  # type: ignore # TODO
            [self.pop, offspring],
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

        results = [
            evaluate_network(individual.genotype.params, io[0], io[1]) for io in ios
        ]
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
