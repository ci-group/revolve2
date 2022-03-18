from __future__ import annotations

import multineat
from dataclasses import dataclass
from revolve2.core.database import Tableable, Database, IncompatibleError
from sqlalchemy.ext.asyncio.session import AsyncSession
from typing import List
from sqlalchemy.future import select
from sqlalchemy.ext.declarative import declarative_base
import sqlalchemy


@dataclass
class Genotype(Tableable):
    genotype: multineat.Genome

    @classmethod
    async def create_tables(cls, database: Database) -> None:
        async with database.engine.begin() as conn:
            await conn.run_sync(Genotype.__DbGenotype.metadata.create_all)

    @classmethod
    def identifying_table(cls) -> str:
        return Genotype.__DbGenotype.__tablename__

    @classmethod
    async def to_database(
        cls, session: AsyncSession, objects: List[Genotype]
    ) -> List[int]:
        dbfitnesses = [
            Genotype.__DbGenotype(serialized_multineat_genome=o.genotype.Serialize())
            for o in objects
        ]
        session.add_all(dbfitnesses)
        await session.flush()
        return [dbfitness.id for dbfitness in dbfitnesses]

    @classmethod
    async def from_database(cls, session: AsyncSession, ids: List[int]) -> Genotype:
        rows = (
            (
                await session.execute(
                    select(Genotype.__DbGenotype).filter(
                        Genotype.__DbGenotype.id.in_(ids)
                    )
                )
            )
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

    __DbBase = declarative_base()

    class __DbGenotype(__DbBase):
        __tablename__ = "cppnwin_genotype"

        id = sqlalchemy.Column(
            sqlalchemy.Integer,
            nullable=False,
            unique=True,
            autoincrement=True,
            primary_key=True,
        )
        serialized_multineat_genome = sqlalchemy.Column(
            sqlalchemy.String, nullable=False
        )
