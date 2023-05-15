"""
Run the example.

A robot brain with a static body will be optimized using CMA-ES.
The genotype of the brain therefore is be a fixed length real-valued vector.
"""

import hashlib
import logging
from typing import Tuple

import cma
import config
import numpy as np
from base import Base
from evaluator import Evaluator
from generation import Generation
from parameters import Parameters
from revolve2.actor_controllers.cpg import CpgNetworkStructure
from revolve2.core.database import OpenMethod, open_database_sqlite
from revolve2.core.modular_robot import Body
from revolve2.core.modular_robot.brains import make_cpg_network_structure_neighbor
from revolve2.core.physics.actor import Actor
from revolve2.standard_resources.modular_robots import gecko
from sqlalchemy.orm import Session


def robot_to_actor_cpg(body: Body) -> Tuple[Actor, CpgNetworkStructure]:
    """
    Convert a body to an actor and get it's corresponding cpg network structure.

    :param body: The body to convert.
    :returns: A tuple of the actor and cpg network structure.
    """
    actor, dof_ids = body.to_actor()
    id_to_hinge = {
        active_hinge.id: active_hinge for active_hinge in body.find_active_hinges()
    }
    active_hinges = [id_to_hinge[dof_id] for dof_id in dof_ids]
    cpg_network_structure = make_cpg_network_structure_neighbor(active_hinges)

    return actor, cpg_network_structure


def main() -> None:
    """Run the program."""
    # set up logging we see all relevant logging messages.
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    # create a unique seed and initialize the random number generator
    # TODO Initialize rng and use it for cma. Currently cma is not reproducable.
    rng_seed = int(
        hashlib.sha256(f"robot_brain_cmaes_seed{config.RNG_SEED}".encode()).hexdigest(),
        16,
    )
    _ = np.random.Generator(np.random.PCG64(rng_seed))

    # intialize the evaluator that will be used to evaluate robots
    evaluator = Evaluator(
        headless=True,
        num_simulators=config.NUM_SIMULATORS,
        simulation_time=config.SIMULATION_TIME,
        sampling_frequency=config.SAMPLING_FREQUENCY,
        control_frequency=config.CONTROL_FREQUENCY,
    )

    # open the database
    dbengine = open_database_sqlite(
        config.DATABASE_FILE,
        open_method=OpenMethod.NOT_EXISTS_AND_CREATE,
    )
    Base.metadata.create_all(dbengine)

    # get the actor and cpg network structure for the body of choice
    actor, cpg_network_structure = robot_to_actor_cpg(gecko())

    # initial parameter values for the brain
    initial_mean = cpg_network_structure.num_connections * [0.5]

    # initial generation. population is empty because it will be defined by CMA-ES
    generation = Generation(
        0,
        [],
    )

    # initialize the cma optimizer
    options = cma.CMAOptions()
    options.set("bounds", [-1.0, 1.0])
    opt = cma.CMAEvolutionStrategy(initial_mean, config.INITIAL_STD, options)

    # run cma for the defined number of generations
    while generation.generation_index < config.NUM_GENERATIONS:
        logging.info(
            f"Generation {generation.generation_index + 1} / {config.NUM_GENERATIONS}."
        )

        # get the sampled solutions(parameters) from cma
        solutions = [tuple(float(p) for p in params) for params in opt.ask()]

        # evaluate them. invert because fitness maximizes, but cma minimizes
        fitnesses = -1.0 * evaluator.evaluate(actor, cpg_network_structure, solutions)

        # tell cma the fitnesses
        opt.tell(solutions, fitnesses)

        logging.info(f"{opt.result.xbest=} {opt.result.fbest=}")

        # make the sampled solutions into a generation
        generation = Generation(
            generation.generation_index + 1,
            [
                Parameters(parameters=solution, fitness=fitness)
                for solution, fitness in zip(solutions, fitnesses)
            ],
        )

        # save the generation
        with Session(dbengine, expire_on_commit=False) as ses:
            ses.add(generation)
            ses.commit()


if __name__ == "__main__":
    main()
