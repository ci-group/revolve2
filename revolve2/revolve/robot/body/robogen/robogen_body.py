from typing import List, Dict

from revolve2.abstract.structural.tree.tree_helper import Coordinate3D, Orientation
from revolve2.revolve.robot.body.body import RobotBody
from revolve2.revolve.robot.body.body_builder import RobotBodyBuilder
from revolve2.revolve.robot.body.robogen.helper.robot_visualizer import generate_matrix, show
from revolve2.revolve.robot.body.robogen.helper.symbolic_measures import MorphologicalMeasureCalculator
from revolve2.revolve.robot.body.robogen.robogen_grammar import RobogenSymbol
from revolve2.revolve.robot.body.robogen.robogen_module import RobogenModule
from revolve2.revolve.robot.development_request import BodyDevelopmentRequest


class RobogenBody(RobotBody):

    def __init__(self, axiom: RobogenModule = RobogenModule()):
        super().__init__()
        self.modules: List[RobogenModule] = [axiom]

    def add(self, module: RobogenModule):
        self.modules.append(module)
        if module in RobogenSymbol.actuators():
            self.actuators.append(module)
        elif module in RobogenSymbol.sensors():
            self.sensors.append(module)

    def expandable_modules(self):
        number_of_orientations = len(Orientation.directions())
        available_modules = []

        for module in self.modules:
            if len(module.children.keys()) < number_of_orientations:
                available_modules.append(module)

        return available_modules

    def visualize(self, path=None):
        body_matrix, connections, length, height = generate_matrix(self.modules)
        show(body_matrix, connections, length, height, path)


class RobogenBodyBuilder(RobotBodyBuilder):

    current_module: RobogenModule
    module_stack: List[RobogenModule]
    hashmap: Dict[Coordinate3D, RobogenModule]

    def __init__(self):
        super().__init__(RobogenBody)

    def _initialize(self):
        self.current_module: RobogenModule = RobogenModule()
        self.module_stack: List[RobogenModule] = []
        self.hashmap: Dict[Coordinate3D, RobogenModule] = {self.current_module.coordinate: self.current_module}

    def create(self, body_development_request: BodyDevelopmentRequest):

        self._initialize()
        body = RobogenBody(self.current_module)

        for symbol in body_development_request.genotype.encoding:

            if self._check_stacking(symbol):
                continue

            module = self._process_symbol(symbol)
            if module is not None:
                body.add(module)

        body.morphological_measures = MorphologicalMeasureCalculator.measure_morphology(body.modules)

        return body

    def _check_stacking(self, symbol):
        if symbol is RobogenSymbol.BRACKET_STASH:
            self.module_stack.append(self.current_module)
            return True

        if symbol is RobogenSymbol.BRACKET_POP:
            if len(self.module_stack) > 0:
                self.current_module = self.module_stack.pop()
                return True

        return False

    def _process_symbol(self, symbol):
        if symbol in RobogenSymbol.orientation():
            self.current_module.move_pointer(symbol.value)
            return None

        # Symbol in RobogenSymbol.modules():
        coordinate = self.current_module.next_coordinate()
        if coordinate not in self.hashmap.keys():
            module = self.current_module.add_child(symbol)
            self.current_module = module
            self.hashmap[coordinate] = self.current_module
            return module
        else:
            self.current_module = self.hashmap[coordinate]
            # print("intersection limits ability to create module")
            return None
