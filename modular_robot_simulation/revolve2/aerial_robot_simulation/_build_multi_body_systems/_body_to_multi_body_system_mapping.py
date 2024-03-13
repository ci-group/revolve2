from dataclasses import dataclass, field

from revolve2.aerial_robot.body.base import Motor, MotorSensor
from revolve2.simulation.scene import SimMotor, UUIDKey


@dataclass(eq=False)
class BodyToMultiBodySystemMapping:
    """Mappings from bodies to multi-body system objects."""

    motor_to_sim_motor: dict[UUIDKey[Motor], SimMotor] = field(
        init=False, default_factory=dict
    )
    motor_sensor_to_sim_moptor: dict[
        UUIDKey[MotorSensor], SimMotor
    ] = field(init=False, default_factory=dict)
