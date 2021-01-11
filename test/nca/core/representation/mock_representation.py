from nca.core.genome.representations.representation import Representation


class MockRepresentation(Representation):
    def compatibility(self, other) -> float:
        pass

    def visit(self, representation_visitor):
        pass

