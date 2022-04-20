from genotype import GenotypeSerializer, develop
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select

from revolve2.core.database import open_async_database_sqlite
from revolve2.core.database.serializers import DbFloat
from revolve2.core.optimization.ea.generic_ea import DbEAOptimizerIndividual
from revolve2.runners.isaacgym import ModularRobotRerunner


async def main() -> None:

    db = open_async_database_sqlite("./database")
    async with AsyncSession(db) as session:
        best_individual = (
            await session.execute(
                select(DbEAOptimizerIndividual, DbFloat)
                .filter(DbEAOptimizerIndividual.fitness_id == DbFloat.id)
                .order_by(DbFloat.value.desc())
            )
        ).first()

        assert best_individual is not None

        print(f"fitness: {best_individual[1].value}")

        genotype = (
            await GenotypeSerializer.from_database(
                session, [best_individual[0].genotype_id]
            )
        )[0]

    rerunner = ModularRobotRerunner()
    await rerunner.rerun(develop(genotype), 5)


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
