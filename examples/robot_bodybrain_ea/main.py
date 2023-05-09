import hashlib
import logging
from typing import List, Tuple, cast

import config
import multineat
import numpy as np
from base import Base
from evaluator import Evaluator
from generation import Generation
from genotype import Genotype
from individual import Individual
from population import Population
from revolve2.core.database import OpenCheck, open_database_sqlite
from revolve2.core.optimization.ea import population_management, selection
from sqlalchemy.orm import Session


def select_parents(
    rng: np.random.Generator,
    population: Population,
    offspring_size: int,
) -> List[Tuple[int, int]]:
    return [
        cast(
            Tuple[int, int],
            selection.multiple_unique(
                2,
                [individual.genotype for individual in population.individuals],
                [individual.fitness for individual in population.individuals],
                lambda _, fitnesses: selection.tournament(rng, fitnesses, k=1),
            ),
        )
        for _ in range(offspring_size)
    ]


def select_survivors(
    rng: np.random.Generator,
    original_population: Population,
    offspring_population: Population,
) -> Population:
    original_survivors, offspring_survivors = population_management.steady_state(
        [i.genotype for i in original_population.individuals],
        [i.fitness for i in original_population.individuals],
        [i.genotype for i in offspring_population.individuals],
        [i.fitness for i in offspring_population.individuals],
        lambda n, genotypes, fitnesses: selection.multiple_unique(
            n,
            genotypes,
            fitnesses,
            lambda _, fitnesses: selection.tournament(rng, fitnesses, k=2),
        ),
    )

    return Population(
        [
            Individual(
                original_population.individuals[i].genotype,
                original_population.individuals[i].fitness,
            )
            for i in original_survivors
        ]
        + [
            Individual(
                offspring_population.individuals[i].genotype,
                offspring_population.individuals[i].fitness,
            )
            for i in offspring_survivors
        ]
    )


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    # create a unique seed and initialize the random number generator
    rng_seed = int(
        hashlib.sha256(
            f"robot_bodybrain_ea_seed{config.RNG_SEED}".encode()
        ).hexdigest(),
        16,
    )
    rng = np.random.Generator(np.random.PCG64(rng_seed))

    evaluator = Evaluator(
        headless=True,
        num_simulators=config.NUM_SIMULATORS,
        simulation_time=config.SIMULATION_TIME,
        sampling_frequency=config.SAMPLING_FREQUENCY,
        control_frequency=config.CONTROL_FREQUENCY,
    )

    # multineat innovation databases
    innov_db_body = multineat.InnovationDatabase()
    innov_db_brain = multineat.InnovationDatabase()

    dbengine = open_database_sqlite(
        config.DATABASE_FILE,
        open_check=OpenCheck.OVERWITE_IF_EXISTS,  # TODO change to not exists after development
    )
    Base.metadata.create_all(dbengine)

    logging.info("Generating initial population.")
    initial_genotypes = [
        Genotype.random(
            innov_db_body=innov_db_body,
            innov_db_brain=innov_db_brain,
            rng=rng,
        )
        for _ in range(config.POPULATION_SIZE)
    ]
    logging.info("Evaluating initial population.")
    initial_fitnesses = evaluator.evaluate(
        [genotype.develop() for genotype in initial_genotypes]
    )
    population = Population(
        [
            Individual(genotype, fitness)
            for genotype, fitness in zip(initial_genotypes, initial_fitnesses)
        ]
    )
    generation = Generation(
        0,
        population,
    )
    logging.info("Saving initial population.")
    with Session(dbengine, expire_on_commit=False) as ses:
        ses.add(generation)
        ses.commit()

    logging.info("Start optimization process.")
    while generation.generation_index < config.NUM_GENERATIONS:
        logging.info(
            f"Generation {generation.generation_index + 1} / {config.NUM_GENERATIONS}."
        )
        parents = select_parents(rng, generation.population, config.OFFSPRING_SIZE)
        offspring_genotypes = [
            Genotype.crossover(
                generation.population.individuals[parent1_i].genotype,
                generation.population.individuals[parent2_i].genotype,
                rng,
            ).mutate(innov_db_body, innov_db_brain, rng)
            for parent1_i, parent2_i in parents
        ]
        offspring_fitnesses = evaluator.evaluate(
            [genotype.develop() for genotype in offspring_genotypes]
        )
        offspring_population = Population(
            [
                Individual(genotype, fitness)
                for genotype, fitness in zip(offspring_genotypes, offspring_fitnesses)
            ]
        )
        survived_population = select_survivors(
            rng,
            generation.population,
            offspring_population,
        )
        generation = Generation(
            generation.generation_index + 1,
            survived_population,
        )
        with Session(dbengine, expire_on_commit=False) as ses:
            ses.add(generation)
            ses.commit()


if __name__ == "__main__":
    main()
