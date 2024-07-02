"""Rerun the best robot between all reserach_experiments."""

import pickle
import config
from genotype import Genotype
from individual import Individual
from sqlalchemy import select
from sqlalchemy.orm import Session

from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.logging import setup_logging


def main() -> None:
    """Perform the rerun."""
    setup_logging()

    # Load the best individual from the database.
    dbengine = open_database_sqlite(
        config.DATABASE_FILE, open_method=OpenMethod.OPEN_IF_EXISTS
    )

    with Session(dbengine) as ses:
        rows = ses.execute(
            select(Genotype, Individual.fitness, Individual.population_id)
            .join_from(Genotype, Individual, Genotype.id == Individual.genotype_id)
            .where(Individual.population_id == 1010)
            .order_by(Individual.fitness.asc())
            .limit(50)
        ).all()
        assert rows is not None

    with open("../csv/median.pickle", "wb") as f:
        pickle.dump(rows[:4], f)

    """for row in rows:
        try:
            genotype = row[0]
            fitness = row[1]

            modular_robot = genotype.develop()

            logging.info(f"Best fitness: {fitness}")

            # Create the evaluator.
            evaluator = Evaluator(headless=False, num_simulators=1)

            # Show the robot.
            evaluator.evaluate([modular_robot])
        except:
            pass
"""

if __name__ == "__main__":
    main()
