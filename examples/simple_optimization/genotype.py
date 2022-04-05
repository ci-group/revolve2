from __future__ import annotations

from dataclasses import dataclass
from random import Random
from typing import List

import sqlalchemy
from item import Item
from phenotype import Phenotype
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.future import select

from revolve2.core.database import IncompatibleError, Serializer


@dataclass
class Genotype:
    items: List[bool]


def random(rng: Random, has_item_prob: float, num_items: int) -> Genotype:
    return Genotype([rng.random() < has_item_prob for _ in range(num_items)])


def develop(genotype: Genotype, items: List[Item], maximum_weight: float) -> Phenotype:
    phenotype = []
    total_weight = 0
    for has_item, item in zip(genotype.items, items):
        if has_item and total_weight + item.weight < maximum_weight:
            phenotype.append(True)
        else:
            phenotype.append(False)

    return Phenotype(phenotype)


class GenotypeSerializer(Serializer[Genotype]):
    @classmethod
    async def create_tables(cls, session: AsyncSession) -> None:
        await (await session.connection()).run_sync(DbGenotype.metadata.create_all)

    @classmethod
    def identifying_table(cls) -> str:
        return DbGenotype.__tablename__

    @classmethod
    async def to_database(
        cls, session: AsyncSession, objects: List[Genotype]
    ) -> List[int]:
        dbobjects = [
            DbGenotype(items="".join(["1" if x else "0" for x in g.items]))
            for g in objects
        ]
        session.add_all(dbobjects)
        await session.flush()
        ids = [
            dbfitness.id for dbfitness in dbobjects if dbfitness.id is not None
        ]  # cannot be none because not nullable. check if only there to silence mypy.
        assert len(ids) == len(objects)  # but check just to be sure
        return ids

    @classmethod
    async def from_database(
        cls, session: AsyncSession, ids: List[int]
    ) -> List[Genotype]:
        rows = (
            (await session.execute(select(DbGenotype).filter(DbGenotype.id.in_(ids))))
            .scalars()
            .all()
        )

        if len(rows) != len(ids):
            raise IncompatibleError()

        id_map = {t.id: t for t in rows}
        items_str = [id_map[id].items for id in ids]
        items_bool = [[item == "1" for item in items] for items in items_str]
        return [Genotype(items) for items in items_bool]


DbBase = declarative_base()


class DbGenotype(DbBase):
    __tablename__ = "genotype"

    id = sqlalchemy.Column(
        sqlalchemy.Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )
    items = sqlalchemy.Column(sqlalchemy.String, nullable=False)
