import math
from typing import Generic, Type, TypeVar

import numpy as np
from numpy.typing import NDArray
from pyrr import Quaternion, Vector3

from .._module import Module
from ._drone_core import DroneCore

TModule = TypeVar("TModule", bound=Module)
TModuleNP = TypeVar("TModuleNP", bound=np.generic)

class DroneBody:
    """Body of a modular robot."""

    _core: DroneCore

    def __init__(self, core: DroneCore) -> None:
        """
        Initialize this object.

        :param core: The core of the body.
        """
        self._core = core

    @classmethod
    def __find_recur(cls, module: Module, module_type: Type[TModule]) -> list[TModule]:
        modules = []
        if isinstance(module, module_type):
            modules.append(module)
        for child in module.children.values():
            modules.extend(cls.__find_recur(child, module_type))
        return modules

    def find_modules_of_type(self, module_type: Type[TModule]) -> list[TModule]:
        """
        Find all Modules of a certain type in the robot.

        :param module_type: The type.
        :return: The list of Modules.
        """
        return self.__find_recur(self._core, module_type)

    @property
    def core(self) -> DroneCore:
        """
        Get the core of the Body.

        :return: The core.
        """
        return self._core