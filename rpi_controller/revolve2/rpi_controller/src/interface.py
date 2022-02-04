from abc import ABC, abstractmethod
from .controller import Controller


class Interface:
    _controller: Controller

    def __init__(self, controller: Controller) -> None:
        self._controller = controller

    @abstractmethod
    async def run(self) -> None:
        pass
