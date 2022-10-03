from __future__ import annotations

from dataclasses import dataclass
from .._individual import Individual
from typing import List, TypeVar, Generic, Type
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.asyncio import AsyncConnection
from sqlalchemy.future import select
from sqlalchemy import Column, Integer
from sqlalchemy.orm import relationship, declarative_base
import sqlalchemy

TGenotype = TypeVar("TGenotype")
TMeasures = TypeVar("TMeasures")


@dataclass
class PopList(Generic[TGenotype, TMeasures]):
    individuals: List[Individual[TGenotype, TMeasures]]

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

        return PopList(new_individuals)

    @staticmethod
    async def prepare_db(
        conn: AsyncConnection,
        genotype_type: Type[TGenotype],
        measures_type: Type[TMeasures],
    ) -> None:
        await genotype_type.prepare_db(conn)
        await measures_type.prepare_db(conn)
        await conn.run_sync(DbBase.metadata.create_all)

    async def to_db(self, ses: AsyncSession) -> Column[Integer]:
        dbpoplist = DbPopList()
        ses.add(dbpoplist)
        await ses.flush()

        items = [
            DbPopListItem(
                pop_list_id=dbpoplist.id,
                index=i,
                genotype_id=(await v.genotype.to_db(ses)),
            )
            for i, v in enumerate(self.individuals)
        ]
        ses.add_all(items)

        dbmeasures = [i.measures.to_row() for i in self.individuals]
        ses.add_all(dbmeasures)

        return dbpoplist.id

    @classmethod
    async def from_db(
        cls: Type[PopList], ses: AsyncSession, id: Column[Integer]
    ) -> PopList:
        dbpoplist = (
            await ses.execute(select(DbPopList).filter(DbPopList.id == id))
        ).scalar_one_or_none()
        # TODO
        return PopList([])


DbBase = declarative_base()


class DbPopList(DbBase):
    __tablename__ = "pop_list"

    id = Column(
        Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )


class DbPopListItem(DbBase):
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
