"""Main script for the example."""

import logging

import config
import multineat
import numpy as np
import numpy.typing as npt
from experiments.data_structures import Base, Experiment, Genotype, Generation, Individual, Population
from experiments.tasks.evaluator_locomotion import Evaluator
from sqlalchemy.engine import Engine
import sys
from sqlalchemy.orm import Session

from revolve2.experimentation.database import OpenMethod, open_database_sqlite
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.optimization.ea import population_management, selection
from revolve2.experimentation.rng import make_rng, seed_from_time, make_rng_time_seed
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom, CpgNetworkStructure
from revolve2.modular_robot.brain.cpg._make_cpg_network_structure_neighbor import \
    active_hinges_to_cpg_network_structure_neighbor

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge, Body

from revde_learner import DifferentialEvolution

N_GEN = 5
INITIAL_POPULATION_INNER_SIZE = 10
RNG = make_rng_time_seed()


def get_robot_from_x(body: Body, genome: npt.NDArray, struct: CpgNetworkStructure) -> ModularRobot:
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=RNG)
    brain._weight_matrix = struct.make_connection_weights_matrix_from_params(genome.tolist())
    return ModularRobot(body=body, brain=brain)


def get_fitnesses(genotypes: list[Genotype], evaluator: Evaluator) -> tuple[list[float], list[float]]:
    amt_ind = len(genotypes)
    structs: list[CpgNetworkStructure] = [None] * amt_ind
    des: list[DifferentialEvolution] = [None] * amt_ind
    robots = [genotype.develop() for genotype in genotypes]

    i = 0
    for tmp in robots:
        struct, _ = active_hinges_to_cpg_network_structure_neighbor(tmp.body.find_modules_of_type(ActiveHinge))
        structs[i] = struct
        des[i] = DifferentialEvolution(x0=np.random.rand(INITIAL_POPULATION_INNER_SIZE, struct.num_connections),
                                       population_size=INITIAL_POPULATION_INNER_SIZE)
        i += 1

    for _ in range(N_GEN):
        test_robots = []
        for tmp, de, struct in zip(robots, des, structs):
            test_robots.extend([get_robot_from_x(tmp.body, genome, struct) for genome in de.x_current])

        all_f = np.array_split(-np.array(evaluator.evaluate(test_robots)), amt_ind)
        j = 0
        for de in des:
            de.f = all_f[j]
            de.new_pop()
            j += 1

    return [-de.f_best_so_far for de in des]


def select_parents(
        rng: np.random.Generator,
        population: Population,
        offspring_size: int,
) -> npt.NDArray[np.float_]:
    """
    Select pairs of parents using a tournament.

    :param rng: Random number generator.
    :param population: The population to select from.
    :param offspring_size: The number of parent pairs to select.
    :returns: Pairs of indices of selected parents. offspring_size x 2 ints.
    """
    return np.array(
        [
            selection.multiple_unique(
                2,
                [individual.genotype for individual in population.individuals],
                [individual.fitness for individual in population.individuals],
                lambda _, fitnesses: selection.tournament(rng, fitnesses, k=1),
            )
            for _ in range(offspring_size)
        ],
    )


def select_survivors(
        rng: np.random.Generator,
        original_population: Population,
        offspring_population: Population,
) -> Population:
    """
    Select survivors using a tournament.

    :param rng: Random number generator.
    :param original_population: The population the parents come from.
    :param offspring_population: The offspring.
    :returns: A newly created population.
    """
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
        individuals=[
                        Individual(
                            genotype=original_population.individuals[i].genotype,
                            fitness=original_population.individuals[i].fitness,
                            age=original_population.individuals[i].age,
                        )
                        for i in original_survivors
                    ]
                    + [
                        Individual(
                            genotype=offspring_population.individuals[i].genotype,
                            fitness=offspring_population.individuals[i].fitness,
                            age=offspring_population.individuals[i].age,
                        )
                        for i in offspring_survivors
                    ]
    )


def find_best_robot(
        current_best: Individual | None, population: list[Individual]
) -> Individual:
    """
    Return the best robot between the population and the current best individual.

    :param current_best: The current best individual.
    :param population: The population.
    :returns: The best individual.
    """
    return max(
        population + [] if current_best is None else [current_best],
        key=lambda x: x.fitness,
    )


def run_experiment(dbengine: Engine) -> None:
    """
    Run an experiment.

    :param dbengine: An openened database with matching initialize database structure.
    """
    logging.info("----------------")
    logging.info("Start experiment")

    # Set up the random number generator.
    rng_seed = seed_from_time()
    rng = make_rng(rng_seed)

    # Create and save the experiment instance.
    experiment = Experiment(rng_seed=rng_seed)
    logging.info("Saving experiment configuration.")
    with Session(dbengine) as session:
        session.add(experiment)
        session.commit()

    # Intialize the evaluator that will be used to evaluate robots.
    evaluator = Eval(headless=True, num_simulators=config.NUM_SIMULATORS)

    # CPPN innovation databases.
    innov_db_body = multineat.InnovationDatabase()
    innov_db_brain = multineat.InnovationDatabase()

    # Create an initial population.
    logging.info("Generating initial population.")
    initial_genotypes = [
        Genotype.random(
            innov_db_body=innov_db_body,
            innov_db_brain=innov_db_brain,
            rng=rng,
        )
        for _ in range(config.POPULATION_SIZE)
    ]

    # Evaluate the initial population.
    logging.info("Evaluating initial population.")
    rng = make_rng_time_seed()

    initial_fitnesses = get_fitnesses(initial_genotypes, evaluator)

    # Create a population of individuals, combining genotype with fitness.
    population = Population(
        individuals=[
            Individual(genotype=genotype, fitness=fitness, age=0)
            for genotype, fitness in zip(
                initial_genotypes, initial_fitnesses, strict=True
            )
        ]
    )

    # Finish the zeroth generation and save it to the database.
    generation = Generation(
        experiment=experiment, generation_index=0, population=population
    )
    logging.info("Saving generation.")
    with Session(dbengine, expire_on_commit=False) as session:
        session.add(generation)
        session.commit()

    # Start the actual optimization process.
    logging.info("Start optimization process.")
    while generation.generation_index < config.NUM_GENERATIONS:
        logging.info(
            f"Generation {generation.generation_index + 1} / {config.NUM_GENERATIONS}."
        )

        # Create offspring.
        parents = select_parents(rng, population, config.OFFSPRING_SIZE)
        offspring_genotypes = [
            Genotype.crossover(
                population.individuals[parent1_i].genotype,
                population.individuals[parent2_i].genotype,
                rng,
            ).mutate(innov_db_body, innov_db_brain, rng)
            for parent1_i, parent2_i in parents
        ]

        offspring_fitnesses = get_fitnesses(offspring_genotypes, evaluator)

        # Make an intermediate offspring population.
        offspring_population = Population(
            individuals=[
                Individual(genotype=genotype, fitness=fitness, age=generation.generation_index + 1,)
                for genotype, fitness in zip(offspring_genotypes, offspring_fitnesses)
            ]
        )

        # Create the next population by selecting survivors.
        population = select_survivors(
            rng,
            population,
            offspring_population,
        )

        # Make it all into a generation and save it to the database.
        generation = Generation(
            experiment=experiment,
            generation_index=generation.generation_index + 1,
            population=population,
        )
        logging.info("Saving generation.")
        with Session(dbengine, expire_on_commit=False) as session:
            session.add(generation)
            session.commit()


def main(objective: str) -> None:
    """Run the program."""
    # Set up logging.
    setup_logging(file_name="tol_log.txt")

    # Open the database, only if it does not already exist.
    dbengine = open_database_sqlite(
        f"{f'_{objective}.'.join(config.DATABASE_FILE.split('.'))}", open_method=OpenMethod.NOT_EXISTS_AND_CREATE
    )
    # Create the structure of the database.
    Base.metadata.create_all(dbengine)

    # Run the experiment several times.
    for _ in range(config.NUM_REPETITIONS):
        run_experiment(dbengine)


if __name__ == "__main__":
    sys.path.append("..")

    objective = sys.argv[1]
    match objective:
        case "l":
            from ..tasks import EvaluatorLocomotion as Eval
        case "s":
            from ..tasks import EvaluatorSearch as Eval
        case "o":
            from ..tasks import EvaluatorObjectManipulation as Eval
        case _:
            raise ValueError(f"Unrecognized objective: {objective}")
    main(objective)


