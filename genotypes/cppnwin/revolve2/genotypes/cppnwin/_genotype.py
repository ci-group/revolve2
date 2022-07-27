from __future__ import annotations

from dataclasses import dataclass
from typing import List

import multineat
from revolve2.core.database import IncompatibleError, Serializer
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select

from .genotype_schema import DbBase, DbGenotype


@dataclass
class Genotype:
    """A generic CPPNWIN genotype."""

    genotype: multineat.Genome


class GenotypeSerializer(Serializer[Genotype]):
    """Serializer for the `Genotype` class."""

    @classmethod
    async def create_tables(cls, session: AsyncSession) -> None:
        """
        Create all tables required for serialization.

        This function commits. TODO fix this
        :param session: Database session used for creating the tables.
        """
        await (await session.connection()).run_sync(DbBase.metadata.create_all)

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
        dbfitnesses = [
            DbGenotype(serialized_multineat_genome=o.genotype.Serialize())
            for o in objects
        ]
        session.add_all(dbfitnesses)
        await session.flush()
        ids = [
            dbfitness.id for dbfitness in dbfitnesses if dbfitness.id is not None
        ]  # cannot be none because not nullable. used to silence mypy
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
        genotypes = [Genotype(multineat.Genome()) for _ in ids]
        for id, genotype in zip(ids, genotypes):
            genotype.genotype.Deserialize(id_map[id].serialized_multineat_genome)
        return genotypes
