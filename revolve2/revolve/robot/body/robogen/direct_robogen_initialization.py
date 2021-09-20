from matplotlib.mathtext import List

from revolve2.nca.core.evolution.conditions.initialization import Initialization

from revolve2.revolve.robot.body.robogen.random_robogen_body import _get_symbol_from_orientation, RandomRobogenBodyBuilder
from revolve2.revolve.robot.body.robogen.robogen_grammar import RobogenSymbol
from revolve2.revolve.robot.body.robogen.robogen_module import RobogenModule


class DirectRepresentationInitialization(Initialization):

    def __init__(self):
        super().__init__()

    def __call__(self, size: int):
        body_builder = RandomRobogenBodyBuilder()

        body = body_builder.develop(size)
        stack = body.modules
        starting = stack.pop(0)
        return self._get_encoding(starting, [])

    def _get_encoding(self, current_module: RobogenModule, encoding: List):

        if current_module.symbol != RobogenSymbol.MODULE_CORE:
            encoding.extend([RobogenSymbol.BRACKET_STASH, current_module.symbol])

        for orientation_key in current_module.children.keys():
            encoding.append(_get_symbol_from_orientation(orientation_key))
            child = current_module.children[orientation_key]
            self._get_encoding(child, encoding)

        if current_module.symbol != RobogenSymbol.MODULE_CORE:
            encoding.append(RobogenSymbol.BRACKET_POP)

        return encoding


if __name__ == "__main__":
    from revolve2.nca.core.genome.genotype import Genotype
    from revolve2.revolve.robot.development_request import BodyDevelopmentRequest
    from revolve2.revolve.robot.body.robogen.robogen_body import RobogenBodyBuilder

    initialization = DirectRepresentationInitialization()

    genotype = Genotype(None)
    genotype.encoding = initialization(3)
    print(genotype.encoding)

    request = BodyDevelopmentRequest(genotype, None)
    robogen_bodybuilder = RobogenBodyBuilder()
    body = robogen_bodybuilder.create(request)

    print(body.modules)
