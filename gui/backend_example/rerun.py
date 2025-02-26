"""Rerun the best robot between all experiments."""

import logging

import sys
from database_components import Genotype, Individual, Experiment
from evaluator import Evaluator
from sqlalchemy import select
from sqlalchemy.orm import Session

from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.logging import setup_logging


def main() -> None:
    """Perform the rerun."""
    setup_logging()

    database_path = f"../resources/databases/{sys.argv[1]}"
    
    dbengine = open_database_sqlite(
        database_path, open_method=OpenMethod.OPEN_IF_EXISTS
    )
    

    with Session(dbengine) as ses:
        row = ses.execute(
            select(Genotype, Individual.fitness)
            .join_from(Genotype, Individual, Genotype.id == Individual.genotype_id)
            .order_by(Individual.fitness.desc())
            .limit(1)
        ).one()
        assert row is not None

        genotype = row[0]
        fitness = row[1]
        terrain_str = ses.execute(
            select(Experiment.terrain)
        ).one()[0]

    print(str(terrain_str))

    logging.info(f"Best fitness: {fitness}")

    # Create the evaluator.
    evaluator = Evaluator(headless=False, num_simulators=1, terrain=terrain_str)

    # Show the robot.
    evaluator.evaluate([genotype])


if __name__ == "__main__":
    main()
