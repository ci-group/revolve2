"""Rerun the best robot between all experiments."""

import logging

import config
from database_components import Genotype, Individual
from evaluator import Evaluator
from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.logging import setup_logging
from sqlalchemy import select
from sqlalchemy.orm import Session


def main() -> None:
    """Perform the rerun."""
    setup_logging()

    # Load the best individual from the database.
    dbengine = open_database_sqlite(
        config.DATABASE_FILE, open_method=OpenMethod.OPEN_IF_EXISTS
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

    logging.info(f"Best fitness: {fitness}")

    # Create the evaluator.
    evaluator = Evaluator(headless=False, num_simulators=1)

    # Show the robot.
    evaluator.evaluate([genotype])


if __name__ == "__main__":
    main()
