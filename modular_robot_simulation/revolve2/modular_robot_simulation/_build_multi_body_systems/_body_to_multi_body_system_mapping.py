from dataclasses import dataclass, field

from revolve2.modular_robot.body.base import ActiveHinge, Motor
from revolve2.modular_robot.body.sensors import (
    ActiveHingeSensor,
    CameraSensor,
    IMUSensor,
)
from revolve2.simulation.scene import JointHinge
from revolve2.simulation.scene import Motor as MotorSim
from revolve2.simulation.scene import MultiBodySystem, UUIDKey
from revolve2.simulation.scene.sensors import CameraSensor as CameraSim
from revolve2.simulation.scene.sensors import IMUSensor as IMUSim


@dataclass(eq=False)
class BodyToMultiBodySystemMapping:
    """Mappings from bodies to multi-body system objects."""

    multi_body_system: MultiBodySystem
    active_hinge_to_joint_hinge: dict[UUIDKey[ActiveHinge], JointHinge] = field(
        init=False, default_factory=dict
    )
    active_hinge_sensor_to_joint_hinge: dict[UUIDKey[ActiveHingeSensor], JointHinge] = (
        field(init=False, default_factory=dict)
    )
    imu_to_sim_imu: dict[UUIDKey[IMUSensor], IMUSim] = field(
        init=False, default_factory=dict
    )
    camera_to_sim_camera: dict[UUIDKey[CameraSensor], CameraSim] = field(
        init=False, default_factory=dict
    )

    motor_to_sim_motor: dict[UUIDKey[Motor], MotorSim] = field(
        init=False, default_factory=dict
    )
