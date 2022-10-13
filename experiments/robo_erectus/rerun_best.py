#!/usr/bin/env python3
"""Visualize and simulate the best robot from the optimization process."""

import os
import argparse
from genotype import GenotypeSerializer, develop
from revolve2.core.modular_robot import ModularRobot
from revolve2.core.database import open_async_database_sqlite
from revolve2.core.database.serializers import DbFloat
from revolve2.core.optimization.ea.generic_ea import DbEAOptimizerIndividual
from revolve2.runners.mujoco import ModularRobotRerunner
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select
from revolve2.runners.mujoco import LocalRunner
from utilities import *


async def main() -> None:
    """Run the script."""
    parser = argparse.ArgumentParser(
        description="reruns simulation for all time best robot"
    )
    parser.add_argument(
        "-t",
        "--time",
        type=float,
        default=1000000,
        help="time (secs) for which to run the simulation",
    )
    parser.add_argument("-l", "--load_latest", action="store_true")
    parser.add_argument("-n", "--run_name", type=str, default="default")
    args = parser.parse_args()

    ensure_dirs(DATABASE_PATH)

    if args.load_latest:
        full_run_name = get_latest_run()
    else:
        full_run_name = find_dir(DATABASE_PATH, args.run_name)

    if full_run_name is None:
        print("Run not found...")
        exit()
    else:
        print(f'Run found - "{full_run_name}"')

    database_dir = os.path.join(DATABASE_PATH, full_run_name)
    analysis_dir = os.path.join(database_dir, ANALYSIS_DIR_NAME)
    ensure_dirs(analysis_dir)

    db = open_async_database_sqlite(database_dir)
    async with AsyncSession(db) as session:
        best_individual = (
            await session.execute(
                select(DbEAOptimizerIndividual, DbFloat)
                .filter(DbEAOptimizerIndividual.fitness_id == DbFloat.id)
                .order_by(DbFloat.value.desc())
            )
        ).first()

        assert best_individual is not None

        print(f"fitness: {best_individual[1].value:0.5f}")

        genotype = (
            await GenotypeSerializer.from_database(
                session, [best_individual[0].genotype_id]
            )
        )[0]

    rerunner = ModularRobotRerunner()

    robot: ModularRobot = develop(genotype)
    env, _ = ModularRobotRerunner.robot_to_env(robot)

    # output env to a MJCF (xml) file (based on LocalRunner.run_batch())
    xml_string = LocalRunner._make_mjcf(env)
    # model = mujoco.MjModel.from_xml_string(xml_string)
    # data = mujoco.MjData(model)
    xml_path = os.path.join(analysis_dir, "best.xml")
    with open(xml_path, "w") as f:
        f.write(xml_string)
    print(f"wrote file: '{xml_path}'")

    # run simulation
    print(f"starting simulation for {args.time} secs...")
    await rerunner.rerun(robot, 60, simulation_time=args.time)


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
