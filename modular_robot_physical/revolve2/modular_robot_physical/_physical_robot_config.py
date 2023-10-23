from __future__ import annotations

import pickle
from dataclasses import dataclass, field

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge


@dataclass
class PhysicalRobotConfig:
    """The configuration for running a physical robot."""

    modular_robot: ModularRobot
    """The Modular Robot Object."""
    hinge_mapping: dict[ActiveHinge, int]
    """Hinge mapping: map each active hinge object to a specific Servo with its ID (int)."""
    simulation_time: int  # (seconds).
    control_frequency: int
    inverse_servos: dict[int, bool] = field(default_factory=lambda: {})
    """
    If a servo is mounted in the wrong direction on the body one can fix it by inversing the action.
    inverse_servos allows you to inverse specific servos with their gpio number as key.
    """

    @staticmethod
    def from_pickle(pickled_object: bytes | str) -> PhysicalRobotConfig:
        """
        Get a PhysicalRobotConfig from pickle bytes.

        :param pickled_object: The bytes object.
        :return: The PhysicalRobotConfig.
        """
        if isinstance(pickled_object, str):
            with open(pickled_object, "rb") as file:
                physical_robot_config: PhysicalRobotConfig = pickle.load(file)
        else:
            obj = pickle.loads(pickled_object)
            physical_robot_config = PhysicalRobotConfig.__new__(PhysicalRobotConfig)
            physical_robot_config.__dict__.update(obj)

        return physical_robot_config
