from pyrevolve.patterns.abstract.singleton import Singleton

class SequentialIdentifier:

    def __init__(self):
        super().__init__()
        self.index = 0

    def current(self):
        return self.index

    def increment(self):
        self.index += 1
        return self.current()

