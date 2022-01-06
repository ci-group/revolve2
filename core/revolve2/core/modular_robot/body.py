from .core import Core


class Body:
    core: Core

    def __init__(self):
        self.core = Core(0.0)
