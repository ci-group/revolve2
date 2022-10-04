from __future__ import annotations

from dataclasses import dataclass
from typing import Generic, List, Type, TypeVar

from sqlalchemy import Column, Integer
from sqlalchemy.ext.asyncio import AsyncConnection
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.orm import declarative_base

from .._db_serializable import DbSerializable
from .._individual import Individual
from .._measures import Measures

TGenotype = TypeVar("TGenotype", bound=DbSerializable)
TMeasures = TypeVar("TMeasures", bound=Measures)


@dataclass
class PopList(Generic[TGenotype, TMeasures]):
    individuals: List[Individual[TGenotype, TMeasures]]

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
        cls: Type[PopList[TGenotype, TMeasures]],
        ses: AsyncSession,
        id: int,
        genotype_type: Type[TGenotype],
        measures_type: Type[TMeasures],
    ) -> PopList[TGenotype, TMeasures]:
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

        genotypes = [await genotype_type.from_db(ses, id) for id in genotypes_ids]
        measures = [await measures_type.from_db(ses, id) for id in measures_ids]

        return PopList(
            [
                Individual(genotype, measures)
                for genotype, measures in zip(genotypes, measures)
            ]
        )


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
    measures_id = Column(Integer, nullable=False)
