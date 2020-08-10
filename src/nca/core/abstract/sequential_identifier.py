from nca.core.abstract.singleton import Singleton


class SequentialIdentifier:

    def __init__(self, start_index: int = 0):
        self.index = start_index

    def id(self):
        self.index += 1
        return self.index


class AgentIdentifier(SequentialIdentifier, metaclass=Singleton):

    def __init__(self):
        super().__init__()


class PopulationIdentifier(SequentialIdentifier, metaclass=Singleton):

    def __init__(self):
        super().__init__()


class GenusIdentifier(SequentialIdentifier, metaclass=Singleton):

    def __init__(self):
        super().__init__()


class NodeIdentifier(SequentialIdentifier, metaclass=Singleton):

    def __init__(self):
        super().__init__()


class SimulatorConnectionIdentifier(SequentialIdentifier, metaclass=Singleton):

    def __init__(self, start_index: int = 1000):
        super().__init__(start_index)
