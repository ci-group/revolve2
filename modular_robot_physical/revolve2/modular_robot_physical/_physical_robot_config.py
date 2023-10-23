from __future__ import annotations

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
