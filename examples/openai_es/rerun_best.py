from optimize import make_body
from optimizer import Brain
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select

from revolve2.core.database import open_async_database_sqlite
from revolve2.core.database.serializers import DbNdarray1xnItem
from revolve2.core.modular_robot import ModularRobot
from revolve2.core.modular_robot.brains import Cpg
from revolve2.core.optimization.ea.openai_es import DbOpenaiESOptimizerIndividual
from revolve2.runners.isaacgym import ModularRobotRerunner


async def main() -> None:

    db = open_async_database_sqlite("./database")
    async with AsyncSession(db) as session:
        best_individual = (
            (
                await session.execute(
                    select(DbOpenaiESOptimizerIndividual).order_by(
                        DbOpenaiESOptimizerIndividual.fitness.desc()
                    )
                )
            )
            .scalars()
            .all()[0]
        )

        best_id = best_individual.individual
        assert best_id is not None

        dbparams = (
            await session.execute(
                select(DbNdarray1xnItem)
                .filter(DbNdarray1xnItem.nparray1xn_id == best_id)
                .order_by(DbNdarray1xnItem.array_index)
            )
        ).scalars()

        params = [p.value for p in dbparams]

        print(f"fitness: {best_individual.fitness}")
        print(f"params: {params}")

        body = make_body()
        active_hinges = body.find_active_hinges()
        connections = Cpg._find_connections(body, active_hinges)
        num_internal_weights = len(active_hinges)

        brain = Brain(
            params[:num_internal_weights],
            params[num_internal_weights:],
        )

        bot = ModularRobot(body, brain)

    rerunner = ModularRobotRerunner()
    await rerunner.rerun(bot, 60)


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
