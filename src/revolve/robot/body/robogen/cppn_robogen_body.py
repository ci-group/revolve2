import os

import numpy as np

from abstract.structural.tree.tree_helper import Orientation
from nca.experiment_manager import ExperimentManager
from revolve.robot.body.robogen.helper.body_measures import SymbolicMeasureCalculator
from revolve.robot.body.robogen.helper.symbolic_measures import MorphologicalMeasureCalculator
from revolve.robot.body.robogen.random_robogen_body import _get_symbol_from_orientation
from revolve.robot.body.robogen.robogen_body import RobogenBodyBuilder, RobogenBody
from revolve.robot.body.robogen.robogen_grammar import RobogenSymbol
from revolve.robot.body.robogen.robogen_module import RobogenModule
from revolve.robot.soul.cppn_adapter import CPPNAdapter


class RandomCPPNRobogenBodyBuilder(RobogenBodyBuilder):

    def __init__(self, number_inputs: int = 3, hidden_units: int = 3, number_outputs: int = 5):
        super().__init__()
        self.number_inputs: int = number_inputs
        self.hidden_units: int = hidden_units
        self.number_outputs: int = number_outputs
        np.random.seed(42)

    def develop(self, size: int = 10):
        self._initialize()

        body = RobogenBody(self.current_module)
        cppn = CPPNAdapter(self.number_inputs, self.hidden_units, self.number_outputs)

        for i in range(size):
            available = body.expandable_modules()

            # pick element
            if len(available) == 0:
                break

            # pick element
            chosen_parent_module: RobogenModule = np.random.choice(available)
            chosen_orientation: Orientation = np.random.choice(chosen_parent_module.neighbor_orientations())

            output = cppn.activate(((chosen_orientation.value + chosen_parent_module.coordinate)*10).tuple())
            chosen_symbol = cppn.classify(output, [RobogenSymbol.MODULE_BLOCK,
                                                   RobogenSymbol.MODULE_BLOCK,
                                                   RobogenSymbol.MODULE_HORIZONTAL_JOINT,
                                                   RobogenSymbol.MODULE_VERTICAL_JOINT,
                                                   None])
            if chosen_symbol is None:
                continue

            orientation = _get_symbol_from_orientation(chosen_orientation)

            self.current_module = chosen_parent_module

            _ = self._process_symbol(orientation)
            module = self._process_symbol(chosen_symbol)
            if module is not None:
                body.add(module)

        body.morphological_measures = MorphologicalMeasureCalculator.measure_morphology(body.modules)
        body.symbolic_measures = SymbolicMeasureCalculator.measure_symbols(body.modules)

        return body

class DFSCPPNRobogenBodyBuilder(RobogenBodyBuilder):

    def __init__(self, number_inputs: int = 3, hidden_units: int = 3, number_outputs: int = 5):
        super().__init__()
        self.number_inputs: int = number_inputs
        self.hidden_units: int = hidden_units
        self.number_outputs: int = number_outputs
        np.random.seed(42)

    def develop(self, size: int = 10):
        self._initialize()

        number_of_orientations = len(Orientation.directions())
        body = RobogenBody(self.current_module)
        cppn = CPPNAdapter(self.number_inputs, self.hidden_units, self.number_outputs)

        available_modules = body.expandable_modules()
        for i in range(size):

            # pick element
            if len(available_modules) == 0:
                break

            # pick element
            chosen_parent_module: RobogenModule = available_modules.pop(0)
            for chosen_orientation in chosen_parent_module.neighbor_orientations():

                output = cppn.activate(((chosen_orientation.value + chosen_parent_module.coordinate)*10).tuple())
                chosen_symbol = cppn.classify(output, [RobogenSymbol.MODULE_BLOCK,
                                                       RobogenSymbol.MODULE_BLOCK,
                                                       RobogenSymbol.MODULE_HORIZONTAL_JOINT,
                                                       RobogenSymbol.MODULE_VERTICAL_JOINT,
                                                       None])
                if chosen_symbol is None:
                    continue

                orientation = _get_symbol_from_orientation(chosen_orientation)

                self.current_module = chosen_parent_module

                _ = self._process_symbol(orientation)
                module = self._process_symbol(chosen_symbol)
                if module is not None:
                    body.add(module)

                    if len(module.children.keys()) < number_of_orientations:
                        available_modules.append(module)

        body.morphological_measures = MorphologicalMeasureCalculator.measure_morphology(body.modules)
        body.symbolic_measures = SymbolicMeasureCalculator.measure_symbols(body.modules)

        return body


if __name__ == "__main__":
    body_builder = RandomCPPNRobogenBodyBuilder()
    for i in range(100):
        body = body_builder.develop(18)
        path = os.path.join(ExperimentManager().folders.experiment_path, "test%d.png" % i)
        body.visualize(path)
