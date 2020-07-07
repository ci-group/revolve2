from pyrevolve.patterns.abstract.singleton import Singleton

@Singleton
class SequentialIdentifier:

    def __init__(self):
        super().__init__()
        self.index = 0

    def current(self):
        return self.index

    def increment(self):
        self.index += 1
        return self.current()


class AgentIdentifier(SequentialIdentifier):

    def __init__(self):
        super().__init__()


class PopulationIdentifier(SequentialIdentifier):

    def __init__(self):
        super().__init__()


class SpeciationIdentifier(SequentialIdentifier):

    def __init__(self):
        super().__init__()
