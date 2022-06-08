from typing import List, Set, Tuple, cast

import multineat

from revolve2.core.modular_robot import ActiveHinge, Body
from revolve2.core.modular_robot.brains import (
    BrainCpgNetworkNeighbour as ModularRobotBrainCpgNetworkNeighbour,
)


class BrainCpgNetworkNeighbourV1(ModularRobotBrainCpgNetworkNeighbour):
    _genotype: multineat.Genome

    def __init__(self, genotype: multineat.Genome):
        self._genotype = genotype

    def _make_weights(
        self,
        active_hinges: List[ActiveHinge],
        connections: List[Tuple[ActiveHinge, ActiveHinge]],
        body: Body,
    ) -> Tuple[List[float], List[float]]:
        brain_net = multineat.NeuralNetwork()
        self._genotype.BuildPhenotype(brain_net)

        internal_weights = [
            self._evaluate_network(
                brain_net,
                [
                    1.0,
                    float(pos.x),
                    float(pos.y),
                    float(pos.z),
                    float(pos.x),
                    float(pos.y),
                    float(pos.z),
                ],
            )
            for pos in [
                body.grid_position(active_hinge) for active_hinge in active_hinges
            ]
        ]

        external_weights = [
            self._evaluate_network(
                brain_net,
                [
                    1.0,
                    float(pos1.x),
                    float(pos1.y),
                    float(pos1.z),
                    float(pos2.x),
                    float(pos2.y),
                    float(pos2.z),
                ],
            )
            for (pos1, pos2) in [
                (body.grid_position(active_hinge1), body.grid_position(active_hinge2))
                for (active_hinge1, active_hinge2) in connections
            ]
        ]

        return (internal_weights, external_weights)

    @staticmethod
    def _evaluate_network(
        network: multineat.NeuralNetwork, inputs: List[float]
    ) -> float:
        network.Input(inputs)
        network.Activate()
        return cast(float, network.Output()[0])  # TODO missing multineat typing
