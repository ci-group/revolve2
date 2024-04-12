from __future__ import annotations

from dataclasses import dataclass, field

from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body.base import ActiveHinge

from ._uuid_key import UUIDKey


@dataclass
class Config:
    """The configuration for running a physical robot."""

    modular_robot: ModularRobot
    """The Modular Robot Object."""
    hinge_mapping: dict[UUIDKey[ActiveHinge], int]
    """Hinge mapping: map each active hinge object to a specific Servo with its ID (int)."""
    initial_hinge_positions: dict[UUIDKey[ActiveHinge], float]
    """Initial positions of the active hinges."""
    run_duration: int
    """Duration to run the brain for in seconds."""
    control_frequency: int
    """Frequency at which to call the brain control functions in seconds. There currently is a bug where if you set the control frequency to (around) 10 or smaller the program might hang. This is most likely a big in pycapnp and once pycapnp v2 is released this is probably resolved."""
    inverse_servos: dict[int, bool] = field(default_factory=dict)
    """
    If a servo is mounted in the wrong direction on the body one can fix it by inversing the action.
    inverse_servos allows you to inverse specific servos with their gpio number as key.
    """
