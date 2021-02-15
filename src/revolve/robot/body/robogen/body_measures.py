import math
from typing import List

from abstract.structural.tree.tree_helper import Coordinate3D
from revolve.robot.body.robogen.robogen_grammar import RobogenSymbol
from revolve.robot.body.robogen.robogen_module import RobogenModule
from simulation.simulation_measures import Measures


class SymbolicMeasures(Measures):

    def __init__(self, blocks, horizontical_joint, vertical_joints):
        measurements: dict = {'blocks': blocks, 'horizontical_joint': horizontical_joint, 'vertical_joints': vertical_joints}
        super().__init__(measurements)

    def __repr__(self):
        return str(self.__dict__)


class SymbolicMeasureCalculator:

    @classmethod
    def measure_symbols(cls, modules: List[RobogenModule]):
        number_of_blocks = cls._count_symbol(modules, RobogenSymbol.MODULE_BLOCK)
        number_of_vertical_joints = cls._count_symbol(modules, RobogenSymbol.MODULE_HORIZONTAL_JOINT)
        number_of_horizontal_joints = cls._count_symbol(modules, RobogenSymbol.MODULE_VERTICAL_JOINT)

        return SymbolicMeasures(number_of_blocks, number_of_vertical_joints, number_of_horizontal_joints)

    @classmethod
    def _count_symbol(cls, modules: List[RobogenModule], symbol: RobogenSymbol):
        count = 0
        for module in modules:
            if module.symbol == symbol:
                count += 1
        return count
