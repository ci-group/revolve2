import multineat  # very important, because this actually imports the multineat dynamic library
from revolve2.analysis.isaacgym import ModularRobotRerunner
from revolve2.core.database.sqlite import Database as DbSqlite
from revolve2.core.optimization.ea.analyzer import Analyzer as EaAnalyzer

from genotype import Genotype


async def main() -> None:

    db = await DbSqlite.create("./database")
    with db.begin_transaction() as txn:
        analyzer = EaAnalyzer(txn, db.root)

        best = max(
            [
                analyzer.individuals[id]
                for id in analyzer.generations[len(analyzer.generations) - 1]
            ],
            key=lambda individual: individual.fitness,
        )

    genotype = Genotype.deserialize(best.genotype)

    rerunner = ModularRobotRerunner()
    await rerunner.rerun(genotype.develop(), 5)


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
