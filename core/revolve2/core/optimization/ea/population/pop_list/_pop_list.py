from __future__ import annotations

from typing import List, Protocol, Type, TypeVar

from sqlalchemy import Column, Integer
from sqlalchemy.ext.asyncio import AsyncConnection
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.orm import declarative_base

from .._individual import Individual
from .._measures import Measures
from .._serializable import Serializable

TGenotype = TypeVar("TGenotype", bound=Serializable)
TMeasures = TypeVar("TMeasures", bound=Measures)


class PopList(Serializable, Protocol[TGenotype, TMeasures]):
    """Interface for the generic PopList class."""

    individuals: List[Individual[TGenotype, TMeasures]]

    def __init__(self, individuals: List[Individual[TGenotype, TMeasures]]) -> None:
        """
        Initialize this object.

        :param individuals: Individuals in the population.
        """
        pass

    @staticmethod
    def from_existing_populations(
        populations: List[PopList[TGenotype, TMeasures]],
        selections: List[List[int]],
        copied_measures: List[str],
    ) -> PopList[TGenotype, TMeasures]:
        """
        Create a population from a set of existing populations using a provided selection from each population and copying the provided measures.

        :param populations: The populations to combine.
        :param selection: The individuals to select from each population.
        :param copied_measures: The measures to copy.
        :returns: The created population.
        """


def PopListTemplate(
    genotype_type: Type[TGenotype], measures_type: Type[TMeasures]
) -> Type[PopList[TGenotype, TMeasures]]:
    """
    Create a PopList type using the provided generic parameters.

    :param genotype_type: Type of the genotype.
    :param measures_type: Type of the measures.
    :returns: The created PopList type.
    """

    class PopListImpl(PopList[TGenotype, TMeasures]):
        """A population stored as a list of individuals."""

        table = DbPopList

        __genotype_type: Type[TGenotype] = genotype_type
        __measures_type: Type[TMeasures] = measures_type

        def __init__(self, individuals: List[Individual[TGenotype, TMeasures]]) -> None:
            self.individuals = individuals

        @staticmethod
        def from_existing_populations(
            populations: List[PopList[TGenotype, TMeasures]],
            selections: List[List[int]],
            copied_measures: List[str],
        ) -> PopList[TGenotype, TMeasures]:
            new_individuals: List[Individual[TGenotype, TMeasures]] = []
            for pop, selection in zip(populations, selections):
                for i in selection:
                    new_ind = Individual(
                        pop.individuals[i].genotype, type(pop.individuals[i].measures)()
                    )
                    for measure in copied_measures:
                        new_ind.measures[measure] = pop.individuals[i].measures[measure]
                    new_individuals.append(new_ind)

            return PopListImpl(new_individuals)

        @classmethod
        async def prepare_db(
            cls,
            conn: AsyncConnection,
        ) -> None:

            await cls.__genotype_type.prepare_db(conn)
            await cls.__measures_type.prepare_db(conn)
            await conn.run_sync(DbBase.metadata.create_all)

        async def to_db(self, ses: AsyncSession) -> int:
            dbpoplist = DbPopList()
            ses.add(dbpoplist)
            await ses.flush()

            measures_ids = [await i.measures.to_db(ses) for i in self.individuals]

            items = [
                DbPopListItem(
                    pop_list_id=dbpoplist.id,
                    index=i,
                    genotype_id=(await v.genotype.to_db(ses)),
                    measures_id=mid,
                )
                for i, (v, mid) in enumerate(zip(self.individuals, measures_ids))
            ]
            ses.add_all(items)

            assert dbpoplist.id is not None
            return dbpoplist.id

        @classmethod
        async def from_db(
            cls,
            ses: AsyncSession,
            id: int,
        ) -> PopListImpl:
            dbitems = (
                await ses.execute(
                    select(DbPopListItem)
                    .filter(DbPopListItem.pop_list_id == id)
                    .order_by(DbPopListItem.index)
                )
            ).scalars()

            genotypes_ids, measures_ids = zip(
                *[(dbitem.genotype_id, dbitem.measures_id) for dbitem in dbitems]
            )

            genotypes = [
                await cls.__genotype_type.from_db(ses, id) for id in genotypes_ids
            ]
            measures = [
                await cls.__measures_type.from_db(ses, id) for id in measures_ids
            ]

            return PopListImpl(
                [
                    Individual(genotype, measures)
                    for genotype, measures in zip(genotypes, measures)
                ]
            )

    return PopListImpl


DbBase = declarative_base()


class DbPopList(DbBase):
    """Main table for the PopList."""

    __tablename__ = "pop_list"

    id = Column(
        Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )


class DbPopListItem(DbBase):
    """Table for items in the PopList."""

    __tablename__ = "pop_list_item"

    id = Column(
        Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )
    pop_list_id = Column(Integer, nullable=False)
    index = Column(
        Integer,
        nullable=False,
    )
    genotype_id = Column(Integer, nullable=False)
    measures_id = Column(Integer, nullable=False)
