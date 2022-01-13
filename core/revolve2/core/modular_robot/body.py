from .core import Core


class Body:
    core: Core

    def __init__(self) -> None:
        self.core = Core(0.0)
