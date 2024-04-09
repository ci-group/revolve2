import copy
import numpy as np
from pyrr import Vector3, Quaternion
from pyrr.quaternion import rotation_axis, create_from_axis_rotation
from revolve2.modular_robot.body.base import Motor
from revolve2.simulation.scene import AABB, MultiBodySystem, Pose, RigidBody, UUIDKey
from revolve2.simulation.scene import Motor as MotorSim
from revolve2.simulation.scene.geometry import GeometryCylinder
from revolve2.simulation.scene.geometry.textures import Texture

from .._body_to_multi_body_system_mapping import BodyToMultiBodySystemMapping
from .._unbuilt_child import UnbuiltChild
from .._convert_color import convert_color
from ._builder import Builder

class MotorBuilder(Builder):
    """A Builder for Cores."""

    _module: Motor
    _rigid_body: RigidBody

    def __init__(
        self,
        module: Motor,
        rigid_body: RigidBody,
    ) -> None:
        """
        Initialize the IMU-Sensor Builder.

        :param sensor: The sensor to be built.
        :param rigid_body: The rigid body for the module to be built on.
        :param pose: The pose of the sensor.
        :param imu_location: The location of the IMU locally.
        """
        
        self._module = module
        self._rigid_body = rigid_body

    def build(
        self,
        multi_body_system: MultiBodySystem,
        body_to_multi_body_system_mapping: BodyToMultiBodySystemMapping,
    ) -> list[UnbuiltChild]:
        """
        Build a module onto the Robot.

        :param multi_body_system: The multi body system of the robot.
        :param body_to_multi_body_system_mapping: A mapping from body to multi-body system
        :return: The next children to be built.
        """

        motor_center_pose = Pose(
            self._module.position,
            self._module.orientation,
        )

        motor = MotorSim(pose=motor_center_pose, control_range=self._module._control_range, gear=self._module._gear)
        body_to_multi_body_system_mapping.motor_to_sim_motor[UUIDKey(self._module)] = motor
        self._rigid_body.motors.add_motor(motor)

        # The physical motors
        self._rigid_body.geometries.append(
            GeometryCylinder(
                pose=motor_center_pose,
                mass=self._module.mass,
                texture=Texture(base_color=convert_color(self._module.color)),
                radius=self._module.frame_size[0],
                length=self._module.frame_size[1],
            )
        )

        # Rotors
        rotor_offset = self._module.orientation * Vector3([0.0, 0.0, self._module.frame_size[1]/2 + self._module.rotor_size[1]/2])
        rotor_pose = Pose(
            self._module.position+rotor_offset,
            self._module.orientation
        )
        self._rigid_body.geometries.append(
            GeometryCylinder(
                pose=rotor_pose,
                mass=0.0,
                texture=Texture(base_color=convert_color(self._module.rotor_color)),
                radius=self._module.rotor_size[0],
                length=self._module.rotor_size[1],
            )
        )

        # Arms
        rot_axis=np.array([0.0, 0.0, 1.0])
        angle = np.pi-np.arctan(np.linalg.norm(np.cross(self._module.position,rot_axis))/np.dot(self._module.position,rot_axis))
        quat = Quaternion(create_from_axis_rotation(np.cross(self._module.position,rot_axis), angle))
        arm_pose = Pose(
            position=self._module.position/2,
            orientation=quat
        )
        arm_length = self._module.position.length
        self._rigid_body.geometries.append(
            GeometryCylinder(
                pose=arm_pose,
                mass=0.0,
                texture=Texture(base_color=convert_color(self._module.arm_color)),
                radius=0.01,
                length=arm_length,
            )
        )
        
        tasks = []
        for sensor in self._module.sensors.get_all_sensors():
            tasks.append(UnbuiltChild(child_object=sensor, rigid_body=self._rigid_body))

        for child in self._module._children.values():
            unbuilt = UnbuiltChild(
                child_object=child,
                rigid_body=self._rigid_body,
            )
            unbuilt.make_pose(
                position=motor_center_pose.position,
                orientation=motor_center_pose.orientation
            )
            tasks.append(unbuilt)
        return tasks