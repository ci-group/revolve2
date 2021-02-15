import math
from typing import List

from abstract.structural.tree.tree_helper import Coordinate3D
from revolve.robot.body.robogen.robogen_grammar import RobogenSymbol
from revolve.robot.body.robogen.robogen_module import RobogenModule
from simulation.simulation_measures import Measures


class MorphologicalMeasures(Measures):

    def __init__(self, branching: float, limbs: float, length_of_limbs: float, coverage: float = 0.0,
                 joints: float = 0.0, proportion: float = 0.0, symmetry: float = 0.0, size: float = 0.0):
        measurements: dict = {'branching': branching, 'limbs': limbs, 'length_of_limbs': length_of_limbs,
                              'coverage': coverage, 'joints': joints, 'proportion': proportion, 'symmetry': symmetry,
                              'size': size}
        super().__init__(measurements)

    def __repr__(self):
        return str(self.__dict__)


class MorphologicalMeasureCalculator:

    @staticmethod
    def _count_connections(modules: List[RobogenModule], number_of_connections: int):
        number_of_leaves: int = 0
        for module in modules:
            if len(module.children.keys()) == number_of_connections - 1:
                number_of_leaves += 1
        return number_of_leaves

    @classmethod
    def measure_morphology(cls, modules: List[RobogenModule], maximum_number_of_modules=20):
        number_of_modules = len(modules)
        connect_to_core = len(modules[0].children)
        number_of_leaves = cls._count_connections(modules, 1)
        number_double_connections = cls._count_connections(modules, 2)

        branching = cls._calculate_branching(number_of_modules, connect_to_core)
        limbs = cls._calculate_limbs(number_of_modules, number_of_leaves)
        length_of_limbs = cls._calculate_length_limbs(number_of_modules, number_double_connections)
        # TODO joints Not correctly Implemented
        joints = cls._count_joints(modules)
        length, width = cls._calculate_span(modules)
        coverage = cls._calculate_coverage(modules, length, width)
        proportion = cls._calculate_proportion(length, width)
        symmetry = cls._calculate_symmetry(modules)
        size = len(modules) / maximum_number_of_modules

        return MorphologicalMeasures(branching, limbs, length_of_limbs, coverage, joints, proportion, symmetry, size)

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
        return 0.0

    @classmethod
    def _count_joints(cls, modules: List[RobogenModule]):
        count = 0
        number_modules = len(modules)
        if number_modules < 3:
            return 0.0

        for module in modules:
            if module.symbol == RobogenSymbol.MODULE_HORIZONTAL_JOINT or \
                    module.symbol == RobogenSymbol.MODULE_VERTICAL_JOINT:
                count += 1
        return count / math.floor((number_modules-1)/2)

    @classmethod
    def _calculate_span(cls, modules):
        min_x = math.inf
        min_y = math.inf
        max_x = -math.inf
        max_y = -math.inf
        # Z coordinate todo

        for module in modules:

            if module.coordinate.x < min_x:
                min_x = module.coordinate.x
            if module.coordinate.x > max_x:
                max_x = module.coordinate.x

            if module.coordinate.y < min_y:
                min_y = module.coordinate.y
            if module.coordinate.y > max_y:
                max_y = module.coordinate.y

        length = (max_x - min_x)
        width = (max_y - min_y)

        # length and width store absolute differences not block size.
        return length + 1, width + 1

    @classmethod
    def _get_coordinates(cls, modules):
        coordinates = []
        for module in modules:
            coordinates.append(module.coordinate)
        return coordinates

    @classmethod
    def _calculate_symmetry(cls, modules):
        """
        Measure symmetry in the xy plane of the robot.
        """
        coordinates = cls._get_coordinates(modules)

        horizontal_mirrored = 0
        horizontal_total = 0
        vertical_mirrored = 0
        vertical_total = 0

        # Calculate xy symmetry in body
        for position in coordinates:
            if position.x != 0:
                horizontal_total += 1
                if Coordinate3D(-position.x, position.y, position.z) in coordinates:
                    horizontal_mirrored += 1
            if position.y != 0:
                vertical_total += 1
                if Coordinate3D(position.x, -position.y, position.z) in coordinates:
                    vertical_mirrored += 1

        horizontal_symmetry = horizontal_mirrored / horizontal_total if horizontal_mirrored > 0 else 0
        vertical_symmetry = vertical_mirrored / vertical_total if vertical_mirrored > 0 else 0

        return max(horizontal_symmetry, vertical_symmetry)

    @classmethod
    def _calculate_proportion(cls, length, width):
        if length >= width:
            return width / float(length)
        elif width > length:
            return length / float(width)

    @classmethod
    def _calculate_coverage(cls, modules, length, width):
        return len(modules) / (length * width)
