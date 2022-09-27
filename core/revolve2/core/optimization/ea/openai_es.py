"""OpenAI ES optimizer and corresponding database model."""

import logging
import pickle
from abc import ABC, abstractmethod
from random import Random
from typing import Optional

import numpy as np
import numpy.typing as npt
import sqlalchemy
from revolve2.core.database import IncompatibleError
from revolve2.core.database.serializers import DbNdarray1xn, Ndarray1xnSerializer
from revolve2.core.optimization import DbId, Process
from sqlalchemy.exc import OperationalError
from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.future import select
from sqlalchemy.orm.exc import MultipleResultsFound, NoResultFound


class OpenaiESOptimizer(ABC, Process):
    """
    OpenAI ES optimizer.

    Based on:
    https://gist.github.com/karpathy/77fbb6a8dac5395f1b73e7a89300318d
    https://openai.com/blog/evolution-strategies/
    """

    @abstractmethod
    async def _evaluate_population(
        self,
        database: AsyncEngine,
        db_id: DbId,
        population: npt.NDArray[np.float_],
    ) -> npt.NDArray[np.float_]:
        """
        Evaluate all individuals in the population, returning their fitnesses.

        :param database: Database that can be used to store anything you want to save from the evaluation.
        :param db_id: Unique identifier in the completely program specifically made for this function call.
        :param population: MxN array with M the population size and N the size of an individual.
        :returns: M long vector with M the population size, representing the fitness of each individual in `population`.
        """

    @abstractmethod
    def _must_do_next_gen(self) -> bool:
        """
        Decide if the optimizer must do another generation.

        :returns: True if it must.
        """

    __database: AsyncEngine
    __db_id: DbId

    __rng: Random

    __population_size: int
    __sigma: float
    __learning_rate: float

    __gen_num: int
    __mean: npt.NDArray[np.float_]  # Nx1 array

    async def ainit_new(
        self,
        database: AsyncEngine,
        session: AsyncSession,
        db_id: DbId,
        rng: Random,
        population_size: int,
        sigma: float,
        learning_rate: float,
        initial_mean: npt.NDArray[np.float_],
    ) -> None:
        """
        Initialize this class async.

        Called when creating an instance using `new`.

        :param database: Database to use for this optimizer.
        :param session: Session to use when saving data to the database during initialization.
        :param db_id: Unique identifier in the completely program specifically made for this optimizer.
        :param rng: Random number generator used in the complete optimization process.
        :param population_size: Size of the population. OpenAI ES parameter.
        :param sigma: Standard deviation. OpenAI ES parameter.
        :param learning_rate: Gain factor for the directional vector. OpenAI ES parameter.
        :param initial_mean: Nx1 array. Initial guess. OpenAI ES Parameter.
        """
        self.__database = database
        self.__db_id = db_id

        self.__rng = rng

        self.__population_size = population_size
        self.__sigma = sigma
        self.__learning_rate = learning_rate

        self.__gen_num = 0
        self.__mean = initial_mean

        await (await session.connection()).run_sync(DbBase.metadata.create_all)
        await Ndarray1xnSerializer.create_tables(session)

        dbmeanid = (await Ndarray1xnSerializer.to_database(session, [self.__mean]))[0]
        dbopt = DbOpenaiESOptimizer(
            db_id=self.__db_id.fullname,
            population_size=self.__population_size,
            sigma=self.__sigma,
            learning_rate=self.__learning_rate,
            initial_mean=dbmeanid,
            initial_rng=pickle.dumps(self.__rng.getstate()),
        )
        session.add(dbopt)

    async def ainit_from_database(
        self,
        database: AsyncEngine,
        session: AsyncSession,
        db_id: DbId,
        rng: Random,
    ) -> bool:
        """
        Try to initialize this class async from a database.

        Called when creating an instance using `from_database`.

        :param database: Database to use for this optimizer.
        :param session: Session to use when loading and saving data to the database during initialization.
        :param db_id: Unique identifier in the completely program specifically made for this optimizer.
        :param rng: Random number generator used in the complete optimization process. Its state will be overwritten with the serialized state from the database.
        :returns: True if the complete object could be deserialized from the database.
        :raises IncompatibleError: In case the database is not compatible with this class.
        """
        self.__database = database
        self.__db_id = db_id

        try:
            opt_row = (
                (
                    await session.execute(
                        select(DbOpenaiESOptimizer).filter(
                            DbOpenaiESOptimizer.db_id == self.__db_id.fullname
                        )
                    )
                )
                .scalars()
                .one()
            )
        except MultipleResultsFound as err:
            raise IncompatibleError() from err
        except (NoResultFound, OperationalError):
            return False

        self.__population_size = opt_row.population_size
        self.__sigma = opt_row.sigma
        self.__learning_rate = opt_row.learning_rate

        db_state = (
            (
                await session.execute(
                    select(DbOpenaiESOptimizerState)
                    .filter(DbOpenaiESOptimizerState.db_id == self.__db_id.fullname)
                    .order_by(DbOpenaiESOptimizerState.gen_num.desc())
                )
            )
            .scalars()
            .first()
        )

        self.__rng = rng  # set state from database below

        if db_state is None:
            self.__gen_num = 0
            db_mean_id = opt_row.initial_mean
            self.__rng.setstate(pickle.loads(opt_row.initial_rng))
        else:
            self.__gen_num = db_state.gen_num
            db_mean_id = db_state.mean
            self.__rng.setstate(pickle.loads(db_state.rng))

        self.__mean = (await Ndarray1xnSerializer.from_database(session, [db_mean_id]))[
            0
        ]

        return True

    async def run(self) -> None:
        """Run the optimizer."""
        while self.__safe_must_do_next_gen():
            rng = np.random.Generator(
                np.random.PCG64(self.__rng.randint(0, 2**63))
            )  # rng is currently not numpy, but this would be very convenient. do this until that is resolved.
            pertubations = rng.standard_normal(
                (self.__population_size, len(self.__mean))
            )
            population = self.__sigma * pertubations + self.__mean

            fitnesses = await self._evaluate_population(
                self.__database,
                self.__db_id,
                population,
            )

            assert fitnesses.shape == (len(population),)
            fitnesses_gaussian = (fitnesses - np.mean(fitnesses)) / np.std(fitnesses)
            self.__mean = self.__mean + self.__learning_rate / (
                self.__population_size * self.__sigma
            ) * np.dot(pertubations.T, fitnesses_gaussian)

            self.__gen_num += 1

            async with AsyncSession(self.__database) as session:
                async with session.begin():
                    db_mean_id = (
                        await Ndarray1xnSerializer.to_database(session, [self.__mean])
                    )[0]

                    dbopt = DbOpenaiESOptimizerState(
                        db_id=self.__db_id.fullname,
                        gen_num=self.__gen_num,
                        mean=db_mean_id,
                        rng=pickle.dumps(self.__rng.getstate()),
                    )

                    session.add(dbopt)

                    db_individual_ids = await Ndarray1xnSerializer.to_database(
                        session, [i for i in population]
                    )

                    dbgens = [
                        DbOpenaiESOptimizerIndividual(
                            db_id=self.__db_id.fullname,
                            gen_num=self.__gen_num,
                            gen_index=index,
                            individual=id,
                            fitness=fitness,
                        )
                        for index, id, fitness in zip(
                            range(len(population)), db_individual_ids, fitnesses
                        )
                    ]

                    session.add_all(dbgens)

                    logging.info(f"Finished generation {self.__gen_num}")

    @property
    def generation_number(self) -> Optional[int]:
        """
        Get the current generation.

        The initial generation is numbered 0.

        :returns: The current generation.
        """
        return self.__gen_num

    def __safe_must_do_next_gen(self) -> bool:
        must_do = self._must_do_next_gen()
        assert type(must_do) == bool
        return must_do


DbBase = declarative_base()


class DbOpenaiESOptimizer(DbBase):
    """Model for the optimizer itself, containing static parameters."""

    __tablename__ = "openaies_optimizer"

    db_id = sqlalchemy.Column(
        sqlalchemy.String,
        nullable=False,
        unique=True,
        primary_key=True,
    )
    population_size = sqlalchemy.Column(sqlalchemy.Integer, nullable=False)
    sigma = sqlalchemy.Column(sqlalchemy.Float, nullable=False)
    learning_rate = sqlalchemy.Column(sqlalchemy.Float, nullable=False)
    initial_mean = sqlalchemy.Column(
        sqlalchemy.Integer, sqlalchemy.ForeignKey(DbNdarray1xn.id), nullable=False
    )
    initial_rng = sqlalchemy.Column(sqlalchemy.PickleType, nullable=False)


class DbOpenaiESOptimizerState(DbBase):
    """State of the optimizer."""

    __tablename__ = "openaies_optimizer_state"

    db_id = sqlalchemy.Column(sqlalchemy.String, nullable=False, primary_key=True)
    gen_num = sqlalchemy.Column(sqlalchemy.Integer, nullable=False, primary_key=True)
    mean = sqlalchemy.Column(
        sqlalchemy.Integer, sqlalchemy.ForeignKey(DbNdarray1xn.id), nullable=False
    )
    rng = sqlalchemy.Column(sqlalchemy.PickleType, nullable=False)


class DbOpenaiESOptimizerIndividual(DbBase):
    """An individual with a fitness which may or may not be assigned."""

    __tablename__ = "openaies_optimizer_individual"

    db_id = sqlalchemy.Column(sqlalchemy.String, nullable=False, primary_key=True)
    gen_num = sqlalchemy.Column(sqlalchemy.Integer, nullable=False, primary_key=True)
    gen_index = sqlalchemy.Column(sqlalchemy.Integer, nullable=False, primary_key=True)
    individual = sqlalchemy.Column(
        sqlalchemy.Integer, sqlalchemy.ForeignKey(DbNdarray1xn.id), nullable=False
    )
    fitness = sqlalchemy.Column(sqlalchemy.Float, nullable=True)
