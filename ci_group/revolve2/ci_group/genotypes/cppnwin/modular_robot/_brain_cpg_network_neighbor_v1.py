from typing import cast

import multineat

from revolve2.modular_robot.body.base import ActiveHinge, Body
from revolve2.modular_robot.brain.cpg import (
    BrainCpgNetworkNeighbor as ModularRobotBrainCpgNetworkNeighbor,
)


class BrainCpgNetworkNeighborV1(ModularRobotBrainCpgNetworkNeighbor):
    """
    A CPG brain based on `ModularRobotBrainCpgNetworkNeighbor` that creates weights from a CPPNWIN network.

    Weights are determined by querying the CPPN network with inputs:
    (hinge1_posx, hinge1_posy, hinge1_posz, hinge2_posx, hinge2_posy, hinge3_posz)
    If the weight in internal, hinge1 and hinge2 position will be the same.
    """

    _genotype: multineat.Genome

    def __init__(self, genotype: multineat.Genome, body: Body):
        """
        Initialize this object.

        :param genotype: A multineat genome used for determining weights.
        :param body: The body of the robot.
        """
        self._genotype = genotype
        super().__init__(body)

    def _make_weights(
        self,
        active_hinges: list[ActiveHinge],
        connections: list[tuple[ActiveHinge, ActiveHinge]],
        body: Body,
    ) -> tuple[list[float], list[float]]:
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
        network: multineat.NeuralNetwork, inputs: list[float]
    ) -> float:
        network.Input(inputs)
        network.ActivateAllLayers()
        return cast(float, network.Output()[0])
