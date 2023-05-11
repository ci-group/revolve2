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
from revolve2.core.database import OpenCheck, open_database_sqlite
from revolve2.core.modular_robot import Body
from revolve2.core.modular_robot.brains import make_cpg_network_structure_neighbor
from revolve2.core.physics.actor import Actor
from revolve2.standard_resources.modular_robots import gecko
from sqlalchemy.orm import Session


def robot_to_actor_cpg(body: Body) -> Tuple[Actor, CpgNetworkStructure]:
    actor, dof_ids = body.to_actor()
    id_to_hinge = {
        active_hinge.id: active_hinge for active_hinge in body.find_active_hinges()
    }
    active_hinges = [id_to_hinge[dof_id] for dof_id in dof_ids]
    cpg_network_structure = make_cpg_network_structure_neighbor(active_hinges)

    return actor, cpg_network_structure


def main() -> None:
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

    evaluator = Evaluator(
        headless=True,
        num_simulators=config.NUM_SIMULATORS,
        simulation_time=config.SIMULATION_TIME,
        sampling_frequency=config.SAMPLING_FREQUENCY,
        control_frequency=config.CONTROL_FREQUENCY,
    )

    dbengine = open_database_sqlite(
        config.DATABASE_FILE,
        open_check=OpenCheck.OVERWITE_IF_EXISTS,  # TODO change to not exists after development
    )
    Base.metadata.create_all(dbengine)

    actor, cpg_network_structure = robot_to_actor_cpg(gecko())

    initial_mean = cpg_network_structure.num_connections * [0.5]

    generation = Generation(
        0,
        [],
    )

    options = cma.CMAOptions
    options.set("bounds", [-1.0, 1.0])
    opt = cma.CMAEvolutionStrategy(initial_mean, config.INITIAL_STD, options)
    while generation.generation_index < config.NUM_GENERATIONS:
        logging.info(
            f"Generation {generation.generation_index + 1} / {config.NUM_GENERATIONS}."
        )

        solutions = [tuple(float(p) for p in params) for params in opt.ask()]
        fitnesses = -1.0 * evaluator.evaluate(actor, cpg_network_structure, solutions)
        opt.tell(solutions, fitnesses)
        opt.disp()

        logging.info(opt.result.xbest)
        logging.info(opt.result.fbest)

        generation = Generation(
            generation.generation_index + 1,
            [
                Parameters(parameters=solution, fitness=fitness)
                for solution, fitness in zip(solutions, fitnesses)
            ],
        )
        with Session(dbengine, expire_on_commit=False) as ses:
            ses.add(generation)
            ses.commit()


if __name__ == "__main__":
    main()
