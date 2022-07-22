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
    genotype: multineat.Genome


class GenotypeSerializer(Serializer[Genotype]):
    @classmethod
    async def create_tables(cls, session: AsyncSession) -> None:
        await (await session.connection()).run_sync(DbBase.metadata.create_all)

    @classmethod
    def identifying_table(cls) -> str:
        return DbGenotype.__tablename__

    @classmethod
    async def to_database(
        cls, session: AsyncSession, objects: List[Genotype]
    ) -> List[int]:
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
