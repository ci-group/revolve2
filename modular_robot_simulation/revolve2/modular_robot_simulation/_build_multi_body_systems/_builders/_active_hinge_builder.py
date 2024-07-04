from pyrr import Quaternion, Vector3
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.simulation.scene import (
    AABB,
    JointHinge,
    MultiBodySystem,
    Pose,
    RigidBody,
    UUIDKey,
)
from revolve2.simulation.scene.geometry import GeometryBox
from revolve2.simulation.scene.geometry.textures import Texture

from .._body_to_multi_body_system_mapping import BodyToMultiBodySystemMapping
from .._convert_color import convert_color
from .._unbuilt_child import UnbuiltChild
from ._builder import Builder


class ActiveHingeBuilder(Builder):
    """A Builder for Hinges."""

    _module: ActiveHinge

    def __init__(self, module: ActiveHinge, rigid_body: RigidBody, slot_pose: Pose):
        """
        Initialize the Active Hinge Builder.

        :param module: The module to be built.
        :param rigid_body: The rigid body for the module to be built on.
        :param slot_pose: The slot pose of the module.
        """
        self._module = module
        self._rigid_body = rigid_body
        self._slot_pose = slot_pose

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
        SERVO_BBOX2_POSITION = Vector3(
            [
                self._module.servo1_bounding_box[0] / 2.0
                + self._module.servo2_bounding_box[0] / 2.0,
                0.0,
                0.0,
            ]
        )

        frame_position = (
            self._slot_pose.position
            + self._slot_pose.orientation
            * Vector3([self._module.frame_offset / 2.0, 0.0, 0.0])
        )

        frame_pose_real = Pose(
            self._slot_pose.position
            + self._slot_pose.orientation
            * Vector3([self._module.frame_bounding_box[0] / 2.0, 0.0, 0.0]),
            self._slot_pose.orientation,
        )
        servo_body_pose = Pose(
            self._rigid_body.initial_pose.position
            + self._rigid_body.initial_pose.orientation
            * (
                frame_position
                + self._slot_pose.orientation
                * Vector3([self._module.servo_offset, 0.0, 0.0])
            ),
            self._rigid_body.initial_pose.orientation * self._slot_pose.orientation,
        )
        joint_pose = Pose(
            self._rigid_body.initial_pose.position
            + self._rigid_body.initial_pose.orientation
            * (
                frame_position
                + self._slot_pose.orientation
                * Vector3([self._module.joint_offset, 0.0, 0.0])
            ),
            self._rigid_body.initial_pose.orientation * self._slot_pose.orientation,
        )

        self._rigid_body.geometries.append(
            GeometryBox(
                pose=frame_pose_real,
                mass=self._module.frame_mass,
                texture=Texture(base_color=convert_color(self._module.color)),
                aabb=AABB(self._module.frame_bounding_box),
            )
        )

        next_rigid_body = RigidBody(
            initial_pose=servo_body_pose,
            static_friction=self._module.static_friction,
            dynamic_friction=self._module.dynamic_friction,
            geometries=[],
        )
        multi_body_system.add_rigid_body(next_rigid_body)

        joint = JointHinge(
            pose=joint_pose,
            rigid_body1=self._rigid_body,
            rigid_body2=next_rigid_body,
            axis=Vector3([0.0, 1.0, 0.0]),
            range=self._module.range,
            effort=self._module.effort,
            velocity=self._module.velocity,
            armature=self._module.armature,
            pid_gain_p=self._module.pid_gain_p,
            pid_gain_d=self._module.pid_gain_d,
        )
        multi_body_system.add_joint(joint)
        body_to_multi_body_system_mapping.active_hinge_to_joint_hinge[
            UUIDKey(self._module)
        ] = joint

        next_rigid_body.geometries.append(
            GeometryBox(
                pose=Pose(Vector3(), Quaternion()),
                mass=self._module.servo1_mass,
                texture=Texture(base_color=convert_color(self._module.color)),
                aabb=AABB(self._module.servo1_bounding_box),
            )
        )
        next_rigid_body.geometries.append(
            GeometryBox(
                pose=Pose(SERVO_BBOX2_POSITION, Quaternion()),
                mass=self._module.servo2_mass,
                texture=Texture(base_color=convert_color(self._module.color)),
                aabb=AABB(self._module.servo2_bounding_box),
            )
        )

        tasks = []
        attachment_point = self._module.attachment_points[self._module.ATTACHMENT]
        child = self._module.children.get(self._module.ATTACHMENT)

        for sensor in self._module.sensors.get_all_sensors():
            tasks.append(UnbuiltChild(child_object=sensor, rigid_body=next_rigid_body))

        if child is not None:
            unbuilt = UnbuiltChild(
                child_object=child,
                rigid_body=next_rigid_body,
            )
            unbuilt.make_pose(attachment_point.offset)
            tasks.append(unbuilt)
        return tasks
