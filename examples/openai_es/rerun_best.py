import math

from optimize import make_body
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select

from revolve2.core.database import open_async_database_sqlite
from revolve2.core.database.serializers import (
    DbNdarray1xn,
    DbNdarray1xnItem,
    Ndarray1xnSerializer,
)
from revolve2.core.modular_robot import ModularRobot
from revolve2.core.modular_robot.brains import (
    BrainCpgNetworkStatic,
    make_cpg_network_structure_neighbour,
)
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

        params = [
            p
            for p in (
                await Ndarray1xnSerializer.from_database(
                    session, [best_individual.individual]
                )
            )[0]
        ]

        print(f"fitness: {best_individual.fitness}")
        print(f"params: {params}")

        body = make_body()

        actor, dof_ids = body.to_actor()
        active_hinges_unsorted = body.find_active_hinges()
        active_hinge_map = {
            active_hinge.id: active_hinge for active_hinge in active_hinges_unsorted
        }
        active_hinges = [active_hinge_map[id] for id in dof_ids]

        cpg_network_structure = make_cpg_network_structure_neighbour(active_hinges)

        initial_state = cpg_network_structure.make_uniform_state(0.5 * math.pi / 2.0)
        weight_matrix = cpg_network_structure.make_weight_matrix_from_params(params)
        dof_ranges = cpg_network_structure.make_uniform_dof_ranges(1.0)
        brain = BrainCpgNetworkStatic(
            initial_state,
            cpg_network_structure.num_cpgs,
            weight_matrix,
            dof_ranges,
        )

        bot = ModularRobot(body, brain)

    rerunner = ModularRobotRerunner()
    await rerunner.rerun(bot, 5)


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
