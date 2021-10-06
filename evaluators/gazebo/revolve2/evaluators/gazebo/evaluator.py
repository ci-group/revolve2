from typing import Dict, List, Union

from revolve2.core.modular_robot.modular_robot import ModularRobot

SimulationResult = Union[
    bool, int, float, Dict[str, "SimulationResult"], List["SimulationResult"]
]


class Evaluator:
    def SimulateModularRobot(self, robot: ModularRobot) -> SimulationResult:
        sdf = ""  # TODO

        return self.SimulateSdf(sdf)

    def SimulateSdf(self, sdf: str) -> SimulationResult:
        return ""  # TODO
