from dataclasses import dataclass, field

from revolve2.simulation.scene import JointHinge, MultiBodySystem, UUIDKey
from revolve2.simulation.scene.sensors import CameraSensor, IMUSensor


@dataclass
class JointHingeMujoco:
    """Information about a MuJoCo hinge joint."""

    id: int
    ctrl_index_position: int
    ctrl_index_velocity: int


@dataclass
class MultiBodySystemMujoco:
    """Information about a MuJoCo body."""

    id: int


@dataclass
class IMUSensorMujoco:
    """Information about a MuJoCo IMU sensor."""

    gyro_id: int
    accelerometer_id: int


@dataclass
class CameraSensorMujoco:
    """Information about a MuJoCo camera sensor."""

    camera_id: int


@dataclass(eq=False)
class AbstractionToMujocoMapping:
    """Data to interpret a MuJoCo model using the simulation abstraction."""

    hinge_joint: dict[UUIDKey[JointHinge], JointHingeMujoco] = field(
        init=False, default_factory=dict
    )

    multi_body_system: dict[UUIDKey[MultiBodySystem], MultiBodySystemMujoco] = field(
        init=False, default_factory=dict
    )

    imu_sensor: dict[UUIDKey[IMUSensor], IMUSensorMujoco] = field(
        init=False, default_factory=dict
    )

    camera_sensor: dict[UUIDKey[CameraSensor], CameraSensorMujoco] = field(
        init=False, default_factory=dict
    )
