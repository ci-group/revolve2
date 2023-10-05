import logging
import math

from pyrr import Quaternion, Vector3
from revolve2.modular_robot.body import ActiveHinge, Body, Brick
from revolve2.modular_robot.body import Color as ModularRobotColor
from revolve2.modular_robot.body import Core, Module
from revolve2.simulation.scene import AABB
from revolve2.simulation.scene import Color as SimulationColor
from revolve2.simulation.scene import JointHinge, MultiBodySystem, Pose, RigidBody
from revolve2.simulation.scene.geometry import GeometryBox

from ._modular_robot_active_hinge_key import ModularRobotActiveHingeKey


class BodyToMultiBodySystemConverter:
    """A tool to convert modular robot bodies to multi-body systems."""

    _STATIC_FRICTION = 1.0
    _DYNAMIC_FRICTION = 1.0

    def convert_robot_body(
        self, body: Body, pose: Pose, translate_z_aabb: bool
    ) -> tuple[MultiBodySystem, dict[ModularRobotActiveHingeKey, JointHinge]]:
        """
        Convert a modular robot body to a multi-body system.

        :param body: The body to convert.
        :param pose: The pose to put the multi-body system in
        :param translate_z_aabb: Whether the robot should be translated upwards so it's T-pose axis-aligned bounding box is exactly on the ground. I.e. if the robot should be placed exactly on the ground. The pose parameters is still added afterwards.
        :returns: The create multi-body system, a mapping from modular robot active hinges to simulation hinge joints.
        """
        actual_pose = pose
        multi_body_system = MultiBodySystem(pose=actual_pose, is_static=False)
        joint_mapping: dict[ModularRobotActiveHingeKey, JointHinge] = {}

        rigid_body = RigidBody(
            initial_pose=Pose(),
            static_friction=self._STATIC_FRICTION,
            dynamic_friction=self._DYNAMIC_FRICTION,
            geometries=[],
        )
        multi_body_system.add_rigid_body(rigid_body)

        self._convert_module(
            multi_body_system=multi_body_system,
            joint_mapping=joint_mapping,
            module=body.core,
            rigid_body=rigid_body,
            slot_pose=Pose(),
        )

        if translate_z_aabb:
            if actual_pose.orientation != Quaternion():
                logging.info(
                    "translate_z_aabb does not yet support non-identity orientation. Orientation ignored for AABB calculation. Robot is probably not positioned as you would like."
                )
            aabb_position, aabb = multi_body_system.calculate_aabb()
            actual_pose.position += Vector3(
                [0.0, 0.0, -aabb_position.z + aabb.size.z / 2.0]
            )

        return multi_body_system, joint_mapping

    def _convert_module(
        self,
        multi_body_system: MultiBodySystem,
        joint_mapping: dict[ModularRobotActiveHingeKey, JointHinge],
        module: Module,
        rigid_body: RigidBody,
        slot_pose: Pose,
    ) -> None:
        if isinstance(module, Core):
            self._convert_core(
                multi_body_system=multi_body_system,
                joint_mapping=joint_mapping,
                module=module,
                rigid_body=rigid_body,
                slot_pose=slot_pose,
            )
        elif isinstance(module, Brick):
            self._convert_brick(
                multi_body_system=multi_body_system,
                joint_mapping=joint_mapping,
                module=module,
                rigid_body=rigid_body,
                slot_pose=slot_pose,
            )
        elif isinstance(module, ActiveHinge):
            self._convert_active_hinge(
                multi_body_system=multi_body_system,
                joint_mapping=joint_mapping,
                module=module,
                rigid_body=rigid_body,
                slot_pose=slot_pose,
            )
        else:
            raise NotImplementedError("Module type not implemented.")

    def _convert_core(
        self,
        multi_body_system: MultiBodySystem,
        joint_mapping: dict[ModularRobotActiveHingeKey, JointHinge],
        module: Core,
        rigid_body: RigidBody,
        slot_pose: Pose,
    ) -> None:
        BOUNDING_BOX = Vector3([0.089, 0.089, 0.0603])  # meter
        MASS = 0.250  # kg
        CHILD_OFFSET = 0.089 / 2.0  # meter

        rigid_body.geometries.append(
            GeometryBox(
                pose=slot_pose,
                mass=MASS,
                color=self._convert_color(module.color),
                aabb=AABB(BOUNDING_BOX),
            )
        )

        for child_index, angle in [
            (Core.FRONT, 0.0),
            (Core.BACK, math.pi),
            (Core.LEFT, math.pi / 2.0),
            (Core.RIGHT, math.pi / 2.0 * 3),
        ]:
            child = module.children[child_index]
            if child is not None:
                child_slot_pose = Pose(
                    position=slot_pose.position
                    + slot_pose.orientation
                    * Quaternion.from_eulers([0.0, 0.0, angle])
                    * Vector3([CHILD_OFFSET, 0.0, 0.0]),
                    orientation=slot_pose.orientation
                    * Quaternion.from_eulers([0.0, 0.0, angle])
                    * Quaternion.from_eulers([child.rotation, 0, 0]),
                )

                self._convert_module(
                    multi_body_system=multi_body_system,
                    joint_mapping=joint_mapping,
                    module=child,
                    rigid_body=rigid_body,
                    slot_pose=child_slot_pose,
                )

    def _convert_brick(
        self,
        multi_body_system: MultiBodySystem,
        joint_mapping: dict[ModularRobotActiveHingeKey, JointHinge],
        module: Brick,
        rigid_body: RigidBody,
        slot_pose: Pose,
    ) -> None:
        BOUNDING_BOX = Vector3([0.06288625, 0.06288625, 0.0603])  # meter
        MASS = 0.030  # kg
        CHILD_OFFSET = 0.06288625 / 2.0  # meter

        brick_center_pose = Pose(
            slot_pose.position
            + slot_pose.orientation * Vector3([BOUNDING_BOX[0] / 2.0, 0.0, 0.0]),
            slot_pose.orientation,
        )

        rigid_body.geometries.append(
            GeometryBox(
                pose=brick_center_pose,
                mass=MASS,
                color=self._convert_color(module.color),
                aabb=AABB(BOUNDING_BOX),
            )
        )

        for child_index, angle in [
            (Brick.FRONT, 0.0),
            (Brick.LEFT, math.pi / 2.0),
            (Brick.RIGHT, math.pi / 2.0 * 3),
        ]:
            child = module.children[child_index]
            if child is not None:
                child_slot_pose = Pose(
                    position=brick_center_pose.position
                    + slot_pose.orientation
                    * Quaternion.from_eulers([0.0, 0.0, angle])
                    * Vector3([CHILD_OFFSET, 0.0, 0.0]),
                    orientation=brick_center_pose.orientation
                    * Quaternion.from_eulers([0.0, 0.0, angle])
                    * Quaternion.from_eulers([child.rotation, 0, 0]),
                )

                self._convert_module(
                    multi_body_system=multi_body_system,
                    joint_mapping=joint_mapping,
                    module=child,
                    rigid_body=rigid_body,
                    slot_pose=child_slot_pose,
                )

    def _convert_active_hinge(
        self,
        multi_body_system: MultiBodySystem,
        joint_mapping: dict[ModularRobotActiveHingeKey, JointHinge],
        module: ActiveHinge,
        rigid_body: RigidBody,
        slot_pose: Pose,
    ) -> None:
        FRAME_BOUNDING_BOX = Vector3([0.018, 0.053, 0.0165891])  # meter
        FRAME_OFFSET = 0.04525
        SERVO1_BOUNDING_BOX = Vector3([0.0583, 0.0512, 0.020])  # meter
        SERVO2_BOUNDING_BOX = Vector3([0.002, 0.053, 0.053])  # meter

        FRAME_MASS = 0.011  # kg
        SERVO1_MASS = 0.058  # kg
        SERVO2_MASS = 0.02  # kg. we simplify by only using the weight of the first box

        SERVO_OFFSET = 0.0299  # meter. distance from frame to servo
        JOINT_OFFSET = 0.0119  # meter. distance from frame to joint

        SERVO_BBOX2_POSITION = Vector3(
            [SERVO1_BOUNDING_BOX[0] / 2.0 + SERVO2_BOUNDING_BOX[0] / 2.0, 0.0, 0.0]
        )

        ATTACHMENT_OFFSET = SERVO1_BOUNDING_BOX[0] / 2.0 + SERVO2_BOUNDING_BOX[0]

        frame_position = slot_pose.position + slot_pose.orientation * Vector3(
            [FRAME_OFFSET / 2.0, 0.0, 0.0]
        )
        frame_pose_real = Pose(
            slot_pose.position
            + slot_pose.orientation * Vector3([FRAME_BOUNDING_BOX[0] / 2.0, 0.0, 0.0]),
            slot_pose.orientation,
        )
        servo_body_pose = Pose(
            rigid_body.initial_pose.position
            + rigid_body.initial_pose.orientation
            * (
                frame_position
                + slot_pose.orientation * Vector3([SERVO_OFFSET, 0.0, 0.0])
            ),
            rigid_body.initial_pose.orientation * slot_pose.orientation,
        )
        joint_pose = Pose(
            rigid_body.initial_pose.position
            + rigid_body.initial_pose.orientation
            * (
                frame_position
                + slot_pose.orientation * Vector3([JOINT_OFFSET, 0.0, 0.0])
            ),
            rigid_body.initial_pose.orientation * slot_pose.orientation,
        )

        rigid_body.geometries.append(
            GeometryBox(
                pose=frame_pose_real,
                mass=FRAME_MASS,
                color=self._convert_color(module.color),
                aabb=AABB(FRAME_BOUNDING_BOX),
            )
        )

        next_rigid_body = RigidBody(
            initial_pose=servo_body_pose,
            static_friction=self._STATIC_FRICTION,
            dynamic_friction=self._DYNAMIC_FRICTION,
            geometries=[],
        )
        multi_body_system.add_rigid_body(next_rigid_body)

        joint = JointHinge(
            pose=joint_pose,
            rigid_body1=rigid_body,
            rigid_body2=next_rigid_body,
            axis=Vector3([0.0, 1.0, 0.0]),
            range=module.RANGE,
            effort=module.EFFORT,
            velocity=module.VELOCITY,
        )
        multi_body_system.add_joint(joint)
        joint_mapping[ModularRobotActiveHingeKey(module)] = joint

        next_rigid_body.geometries.append(
            GeometryBox(
                pose=Pose(Vector3(), Quaternion()),
                mass=SERVO1_MASS,
                color=self._convert_color(module.color),
                aabb=AABB(SERVO1_BOUNDING_BOX),
            )
        )
        next_rigid_body.geometries.append(
            GeometryBox(
                pose=Pose(SERVO_BBOX2_POSITION, Quaternion()),
                mass=SERVO2_MASS,
                color=self._convert_color(module.color),
                aabb=AABB(SERVO2_BOUNDING_BOX),
            )
        )

        child = module.children[ActiveHinge.ATTACHMENT]
        if child is not None:
            rotation = Quaternion.from_eulers([child.rotation, 0.0, 0.0])
            child_slot_pose = Pose(
                rotation * Vector3([ATTACHMENT_OFFSET, 0.0, 0.0]), rotation
            )

            self._convert_module(
                multi_body_system=multi_body_system,
                joint_mapping=joint_mapping,
                module=child,
                rigid_body=next_rigid_body,
                slot_pose=child_slot_pose,
            )

    @staticmethod
    def _convert_color(color: ModularRobotColor) -> SimulationColor:
        return SimulationColor(color.red, color.green, color.blue, color.alpha)
