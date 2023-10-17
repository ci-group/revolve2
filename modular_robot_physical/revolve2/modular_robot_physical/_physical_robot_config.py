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

    @staticmethod
    def from_pickle(pickled_object: bytes) -> PhysicalRobotConfig:
        physical_robot_config = PhysicalRobotConfig.__new__(PhysicalRobotConfig)
        physical_robot_config.__dict__.update(pickle.loads(pickled_object))
        return physical_robot_config
