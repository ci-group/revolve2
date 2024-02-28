from dataclasses import dataclass, field

from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.body.sensors import ActiveHingeSensor
from revolve2.simulation.scene import JointHinge, MultiBodySystem, UUIDKey
from revolve2.simulation.scene.sensors import IMUSensor


@dataclass(eq=False)
class BodyToMultiBodySystemMapping:
    """Mappings from bodies to multi-body system objects."""

    core_imu: IMUSensor
    multi_body_system: MultiBodySystem
    active_hinge_to_joint_hinge: dict[UUIDKey[ActiveHinge], JointHinge] = field(
        init=False, default_factory=dict
    )
    active_hinge_sensor_to_joint_hinge: dict[
        UUIDKey[ActiveHingeSensor], JointHinge
    ] = field(init=False, default_factory=dict)
