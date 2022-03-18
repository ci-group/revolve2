from genotype import Genotype
from revolve2.analysis.isaacgym import ModularRobotRerunner
from revolve2.core.database import open_database_sqlite
from sqlalchemy.future import select
from revolve2.core.optimization.ea.evolutionary_optimizer_schema import (
    DbEvolutionaryOptimizerIndividual,
)
from fitness import DbFitness
from optimizer import develop


async def main() -> None:

    db = open_database_sqlite("./database")
    async with db.session() as session:
        best_individual = (
            await session.execute(
                select(DbEvolutionaryOptimizerIndividual, DbFitness)
                .filter(DbEvolutionaryOptimizerIndividual.fitness_id == DbFitness.id)
                .order_by(DbFitness.fitness.desc())
            )
        ).first()

        print(f"fitness: {best_individual[1].fitness}")

        genotype = (
            await Genotype.from_database(session, [best_individual[0].genotype_id])
        )[0]

    rerunner = ModularRobotRerunner()
    await rerunner.rerun(develop(genotype), 5)


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
