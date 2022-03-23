from genotype import Genotype, develop
from revolve2.analysis.isaacgym import ModularRobotRerunner
from sqlalchemy.future import select

from revolve2.core.database import open_database_sqlite
from revolve2.core.optimization.ea.evolutionary_optimizer_schema import (
    DbEvolutionaryOptimizerIndividual,
)
from revolve2.core.optimization.ea.fitness_float_schema import DbFitnessFloat


async def main() -> None:

    db = open_database_sqlite("./database")
    async with db.session() as session:
        best_individual = (
            await session.execute(
                select(DbEvolutionaryOptimizerIndividual, DbFitnessFloat)
                .filter(
                    DbEvolutionaryOptimizerIndividual.fitness_id == DbFitnessFloat.id
                )
                .order_by(DbFitnessFloat.fitness.desc())
            )
        ).first()

        assert best_individual is not None

        print(f"fitness: {best_individual[1].fitness}")

        genotype = (
            await Genotype.from_database(session, [best_individual[0].genotype_id])
        )[0]

    rerunner = ModularRobotRerunner()
    await rerunner.rerun(develop(genotype), 5)


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
