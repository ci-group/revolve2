
from revolve.robot.robogen.robogen_grammar import RobogenSymbol


class RobogenWord:

    def __init__(self, module: RobogenSymbol, orientation: RobogenSymbol = RobogenSymbol.ORIENTATION_TOP):
        self.module = module
        self.orientation = orientation

    def __hash__(self):
        return hash(hash(self.module) + hash(self.orientation))

    def __eq__(self, other):
        return self.module == other.module and self.orientation == other.orientation

    def __repr__(self):
        return self.module.name + " " + self.orientation.name

    def symbols(self):
        return [self.orientation, self.module]

    @classmethod
    def words(self):
        words = []
        for module in RobogenSymbol.modules():
            for orientation in RobogenSymbol.orientation():
                 words.append(RobogenWord(module, orientation))
        return words
