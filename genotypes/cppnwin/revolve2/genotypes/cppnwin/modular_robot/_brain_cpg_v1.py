from typing import List, Tuple, cast

import multineat

from revolve2.core.modular_robot import AnalyzerModule
from revolve2.core.modular_robot.brains import Cpg as ModularRobotBrainCpg


class BrainCpgV1(ModularRobotBrainCpg):
    _genotype: multineat.Genome

    def __init__(self, genotype: multineat.Genome):
        self._genotype = genotype

    def _make_weights(
        self,
        active_hinges: List[AnalyzerModule],
        connections: List[Tuple[AnalyzerModule, AnalyzerModule]],
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
            for pos in [active_hinge.grid_position() for active_hinge in active_hinges]
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
                (active_hinge1.grid_position(), active_hinge2.grid_position())
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
