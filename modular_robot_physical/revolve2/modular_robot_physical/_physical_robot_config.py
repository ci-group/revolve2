from __future__ import annotations

import pickle
from dataclasses import dataclass

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge


@dataclass
class PhysicalRobotConfig:
    """The configuration for running a physical robot."""

    modular_robot: ModularRobot  # the modular robot object.
    hinge_mapping: dict[ActiveHinge, int]  # which servo is triggered by which hinge.
    simulation_time: int  # in seconds.
    control_frequency: int
    inverse_servos: bool

    @staticmethod
    def from_pickle(pickled_object: bytes| str) -> PhysicalRobotConfig:
        """
        Get a PhysicalRobotConfig from pickle bytes.

        :param pickled_object: The bytes object.
        :return: The PhysicalRobotConfig.
        """
        if isinstance(pickled_object, str):
            with open(pickled_object, "rb") as file:
                obj = pickle.loads(file)
        else:
            obj = pickle.loads(pickled_object)

        physical_robot_config = PhysicalRobotConfig.__new__(PhysicalRobotConfig)
        physical_robot_config.__dict__.update(obj)
        return physical_robot_config
