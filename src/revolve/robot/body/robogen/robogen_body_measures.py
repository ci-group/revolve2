import math
from typing import List

from revolve.robot.body.robogen.robogen_module import RobogenModule


class MorphologicalMeasures:

    def __init__(self, branching: float, limbs: float, length_of_limbs: float, coverage: float = 0.0, joints: float = 0.0, proportion: float = 0.0, symmetry: float = 0.0, size: float = 0.0):
        self.branching: float = branching
        self.limbs: float = limbs
        self.length_of_limbs: float = length_of_limbs
        self.coverage: float = coverage
        self.joints: float = joints
        self.proportion: float = proportion
        self.symmetry: float = symmetry
        self.size: float = size


class MorphologicalMeasureCalculator:

    @staticmethod
    def _count_connections(modules: List[RobogenModule], number_of_connections: int):
        number_of_leaves: int = 0
        for module in modules:
            if module.children == number_of_connections - 1:
                number_of_leaves += 1
        return number_of_leaves

    @classmethod
    def measure_morphology(cls, modules: List[RobogenModule]):
        number_of_modules = len(modules)
        connect_to_core = len(modules[0].children)
        number_of_leaves = cls._count_connections(modules, 1)
        number_double_connections = cls._count_connections(modules, 2)

        branching = cls._calculate_branching(number_of_modules, connect_to_core)
        limbs = cls._calculate_limbs(number_of_modules, number_of_leaves)
        length_of_limbs = cls._calculate_length_limbs(number_of_modules, number_double_connections)
        #expression_efficiency = len(robot_body.modules) / len(encoding)

        measures = MorphologicalMeasures(branching, limbs, length_of_limbs)
        return measures

    @staticmethod
    def _calculate_branching(number_of_modules: int, connected_to_core: int) -> float:
        max_branching = math.floor((number_of_modules - 2) / 3)

        if number_of_modules >= 5:
            return connected_to_core / max_branching
        else:
            return 0.0

    @staticmethod
    def _calculate_limbs(number_of_modules: int, number_of_leaves: int):
        max_limbs = number_of_modules - 1
        if number_of_modules >= 6:
            max_limbs = 2 * math.floor((number_of_modules-6)/3) + (number_of_modules-6) % 3 + 4

        if max_limbs > 0:
            return number_of_leaves / max_limbs
        else:
            return 0.0

    @staticmethod
    def _calculate_length_limbs(number_of_modules, number_double_connections):
        if number_of_modules >= 3:
            return number_double_connections / (number_of_modules - 2)  # TODO
