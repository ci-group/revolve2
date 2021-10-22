from abc import ABC, abstractmethod
from typing import Optional

from revolve2.core.physics_robot import PhysicsRobot

from ._gz_pool import _GzPool


class Controller(ABC):
    @abstractmethod
    def update() -> None:
        pass


class Environment:
    """
    One-off environment to simulate and control one physics_robot at a time in a Gazebo environment.
    """

    _gz_pool: _GzPool
    _robot: Optional[PhysicsRobot]
    _controller: Optional[Controller]

    def __init__(self, gz_pool: _GzPool):
        self._gz_pool = gz_pool
        self._robot = None
        self._controller = None

    @property.setter
    def robot(self, robot: PhysicsRobot) -> None:
        self._robot = robot

    @property.setter
    def controller(self, controller: Controller) -> None:
        self._controller = controller

    async def run(self) -> None:  # TODO result
        if self._robot is None:
            raise RuntimeError("Robot must be set before running.")
        if self._controller is None:
            raise RuntimeError("Controller must be set before running.")

        sim = await self._gz_pool.borrow_simulator()
