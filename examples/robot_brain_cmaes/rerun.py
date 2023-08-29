"""Rerun a robot with given body and parameters."""

import config
import numpy as np
from evaluator import Evaluator
from revolve2.core.modular_robot.brains import (
    body_to_actor_and_cpg_network_structure_neighbour,
)

# These are set of parameters that we optimized using CMA-ES.
# You can copy your own parameters from the optimization output log.
PARAMS = np.array(
    [
        0.96349864,
        0.71928482,
        0.97834176,
        0.90804766,
        0.69150098,
        0.48491278,
        0.40755897,
        0.99818664,
        0.9804162,
        -0.34097883,
        -0.01808513,
        0.76003573,
        0.66221044,
    ]
)


def main() -> None:
    """Perform the rerun."""
    _, cpg_network_structure = body_to_actor_and_cpg_network_structure_neighbour(
        config.BODY
    )

    evaluator = Evaluator(
        headless=False,
        num_simulators=1,
        cpg_network_structure=cpg_network_structure,
        body=config.BODY,
    )
    evaluator.evaluate([PARAMS])


if __name__ == "__main__":
    main()
