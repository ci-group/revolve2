from pyrevolve.shared.abstract.singleton import Singleton


class SequentialIdentifier:

    def __init__(self):
        self.index = 0

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
