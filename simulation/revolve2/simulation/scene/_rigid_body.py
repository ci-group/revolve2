import uuid

from pyrr import Matrix33, Quaternion, Vector3

from ._pose import Pose
from .geometry import Geometry, GeometryBox
from .sensors import CameraSensor, IMUSensor, Sensor


class _AttachedSensors:
    """An object storing attached Sensors."""

    _imu_sensors: list[IMUSensor]
    _camera_sensors: list[CameraSensor]

    def __init__(self) -> None:
        """Initialize this object."""
        self._imu_sensors = []
        self._camera_sensors = []

    def add_sensors(self, sensors: list[Sensor]) -> None:
        """
        Add sensors to the attached sensors object.

        :param sensors: The sensors.
        :raises KeyError: If sensor is not implemented.
        """
        for sensor in sensors:
            match sensor:
                case IMUSensor():
                    self._imu_sensors.append(sensor)
                case CameraSensor():
                    self._camera_sensors.append(sensor)
                case _:
                    raise KeyError(
                        f"Sensor of type {type(sensor)} is not defined for _rigid_body._AttachedSensors"
                    )

    @property
    def imu_sensors(self) -> list[IMUSensor]:
        """
        Return the IMU sensors of the rigid body.

        :return: The IMU sensors.
        """
        return self._imu_sensors

    @property
    def camera_sensors(self) -> list[CameraSensor]:
        """
        Return the camera sensors of the rigid body.

        :return: The cameras sensors.
        """
        return self._camera_sensors


class RigidBody:
    """A collection of geometries and physics parameters."""

    _uuid: uuid.UUID

    @property
    def uuid(self) -> uuid.UUID:
        """
        Get the uuid.

        :returns: The uuid.
        """
        return self._uuid

    initial_pose: Pose
    static_friction: float
    dynamic_friction: float
    geometries: list[Geometry]
    sensors: _AttachedSensors

    def __init__(
        self,
        initial_pose: Pose,
        static_friction: float,
        dynamic_friction: float,
        geometries: list[Geometry],
    ) -> None:
        """
        Initialize the rigid body object.

        :param initial_pose: The Initial pose of the rigid body. Relative to its parent multi-body system.
        :param static_friction: Static friction of the body.
        :param dynamic_friction: Dynamic friction of the body.
        :param geometries: Geometries describing the shape of the body.
        :param sensors: The sensors attached to this rigid body.
        """
        self._uuid = uuid.uuid1()
        self.initial_pose = initial_pose
        self.static_friction = static_friction
        self.dynamic_friction = dynamic_friction
        self.geometries = geometries

        self.sensors = _AttachedSensors()

    def mass(self) -> float:
        """Get mass of the rigid body.

        :returns: The mass.
        """
        return sum(geometry.mass for geometry in self.geometries)

    def center_of_mass(self) -> Vector3:
        """
        Calculate the center of mass in the local reference frame of this rigid body.

        If no geometry has mass, the average position of all geometries is returned, unweighted.

        :returns: The center of mass.
        """
        if self.mass() == 0:
            return sum(
                geometry.mass * geometry.pose.position for geometry in self.geometries
            ) / len(self.geometries)
        else:
            return (
                sum(
                    geometry.mass * geometry.pose.position
                    for geometry in self.geometries
                )
                / self.mass()
            )

    def inertia_tensor(self) -> Matrix33:
        """
        Calculate the inertia tensor in the local reference frame of this rigid body.

        Only box geometries are currently supported.

        :returns: The inertia tensor.
        :raises ValueError: If one of the geometries is not a box.
        """
        com = self.center_of_mass()
        inertia = Matrix33()

        for geometry in self.geometries:
            if geometry.mass == 0:
                continue

            if not isinstance(geometry, GeometryBox):
                raise ValueError(
                    "Geometries with non-zero mass other than box not yet supported."
                )

            # calculate inertia in local coordinates
            local_inertia = Matrix33()
            local_inertia[0][0] += (
                geometry.mass
                * (geometry.aabb.size.y**2 + geometry.aabb.size.z**2)
                / 12.0
            )
            local_inertia[1][1] += (
                geometry.mass
                * (geometry.aabb.size.x**2 + geometry.aabb.size.z**2)
                / 12.0
            )
            local_inertia[2][2] += (
                geometry.mass
                * (geometry.aabb.size.x**2 + geometry.aabb.size.y**2)
                / 12.0
            )

            # convert to global coordinates
            translation = Matrix33()
            translation[0][0] += geometry.mass * (
                (geometry.pose.position.y - com.y) ** 2
                + (geometry.pose.position.z - com.z) ** 2
            )
            translation[1][1] += geometry.mass * (
                (geometry.pose.position.x - com.x) ** 2
                + (geometry.pose.position.z - com.z) ** 2
            )
            translation[2][2] += geometry.mass * (
                (geometry.pose.position.x - com.x) ** 2
                + (geometry.pose.position.y - com.y) ** 2
            )

            ori_as_mat = self._quaternion_to_rotation_matrix(geometry.pose.orientation)
            global_inertia = (
                ori_as_mat * local_inertia * ori_as_mat.transpose() + translation
            )
            # add to rigid body inertia tensor
            inertia += global_inertia

        return inertia

    @staticmethod
    def _quaternion_to_rotation_matrix(quat: Quaternion) -> Matrix33:
        # https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/

        q0 = quat.x
        q1 = quat.y
        q2 = quat.z
        q3 = quat.w

        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)

        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)

        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1

        return Matrix33([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
