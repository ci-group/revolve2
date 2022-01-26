from revolve2.core.database import database_error
from revolve2.core.database.sqlite import Database as DbSqlite
from revolve2.core.optimization.ea.analyzer import Analyzer as EaAnalyzer

from genotype import Genotype


async def main() -> None:
    db = await DbSqlite.create("./database")

    with db.begin_transaction() as txn:
        analyzer = EaAnalyzer(txn, db.root)

        print(f"pop size: {analyzer.population_size}")
        print(f"offspring size: {analyzer.offspring_size}")

        for generation in analyzer.generations:
            print(f"-----------------\nGen {generation.gen_num}")
            for id in generation:
                print(f"id: {id}")

        for individual in analyzer.individuals:
            print(f"-----------------\nIndividual {individual.id}")
            if individual.parents is None:
                print("Gen 0, no parents.")
            else:
                for parent in individual.parents:
                    print(f"parent {parent}")

        best = max(
            [
                analyzer.individuals[id]
                for id in analyzer.generations[len(analyzer.generations) - 1]
            ],
            key=lambda individual: individual.fitness,
        )
        fitness = best.fitness
        genotype = Genotype.deserialize(best.genotype)
        print(f"---------------\nbest from last gen: {best.id} fitness: {fitness}")


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
