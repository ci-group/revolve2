"""Knapsack genotype, functions to work with it, and a serializer."""

from __future__ import annotations

from dataclasses import dataclass
from random import Random
from typing import List

import sqlalchemy
from item import Item
from phenotype import Phenotype
from revolve2.core.database import IncompatibleError, Serializer
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.future import select


@dataclass
class Genotype:
    """
    Genotype that consists of a binary string representing the items in a knapsack.

    More items can be in this string than would actually fit in the knapsack.
    """

    items: List[bool]


def random(rng: Random, has_item_prob: float, num_items: int) -> Genotype:
    """
    Create a random genotype.

    :param rng: Random number generator.
    :param has_item_prob: Probability for each item that is enabled in the genotype(is in the knapsack).
    :param num_items: The number of items that exist.
    :returns: The created genotype.
    """
    return Genotype([rng.random() < has_item_prob for _ in range(num_items)])


def develop(genotype: Genotype, items: List[Item], maximum_weight: float) -> Phenotype:
    """
    Create a knapsack from a knapsack genotype.

    From left to right for each enabled item in the genotype bitstring,
    adds the item to the knapsack if it would be not become too heavy.

    :param genotype: Knapsack genotype.
    :param items: All possible items.
    :param maximum_weight: Maximum weight of the knapsack.
    :returns: A created knapsack.
    """
    phenotype = []
    total_weight = 0
    for has_item, item in zip(genotype.items, items):
        if has_item and total_weight + item.weight < maximum_weight:
            phenotype.append(True)
        else:
            phenotype.append(False)

    return Phenotype(phenotype)


class GenotypeSerializer(Serializer[Genotype]):
    """Serializer for storing knapsack genotypes."""

    @classmethod
    async def create_tables(cls, session: AsyncSession) -> None:
        """
        Create all tables required for serialization.

        This function commits. TODO fix this
        :param session: Database session used for creating the tables.
        """
        await (await session.connection()).run_sync(DbGenotype.metadata.create_all)

    @classmethod
    def identifying_table(cls) -> str:
        """
        Get the name of the primary table used for storage.

        :returns: The name of the primary table.
        """
        return DbGenotype.__tablename__

    @classmethod
    async def to_database(
        cls, session: AsyncSession, objects: List[Genotype]
    ) -> List[int]:
        """
        Serialize the provided objects to a database using the provided session.

        :param session: Session used when serializing to the database. This session will not be committed by this function.
        :param objects: The objects to serialize.
        :returns: A list of ids to identify each serialized object.
        """
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
        """
        Deserialize a list of objects from a database using the provided session.

        :param session: Session used for deserialization from the database. No changes are made to the database.
        :param ids: Ids identifying the objects to deserialize.
        :returns: The deserialized objects.
        :raises IncompatibleError: In case the database is not compatible with this serializer.
        """
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
    """Stores knapsack bitstrings as character strings."""

    __tablename__ = "genotype"

    id = sqlalchemy.Column(
        sqlalchemy.Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )
    items = sqlalchemy.Column(sqlalchemy.String, nullable=False)
