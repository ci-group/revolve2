import math
from typing import cast

from pyrr import Quaternion, Vector3
from revolve2.core.physics_robot import Collision, Joint, RigidBody, Visual

from ..physics_robot import PhysicsRobot, RigidBody
from .active_hinge import ActiveHinge
from .brick import Brick
from .core import Core
from .modular_robot import ModularRobot
from .module import Module


def to_physics_robot(modular_robot: ModularRobot) -> PhysicsRobot:
    return _PhysicsRobotBuilder().build(modular_robot)


class _PhysicsRobotBuilder:
    robot: PhysicsRobot

    def build(self, modular_robot: ModularRobot) -> PhysicsRobot:
        self.robot = PhysicsRobot([], [])

        origin_body = RigidBody("origin", Vector3(), Quaternion())
        self.robot.bodies.append(origin_body)

        self._make_module(
            modular_robot.body.core, origin_body, "origin", Vector3(), Quaternion()
        )

        return self.robot

    def _make_module(
        self,
        module: Module,
        body: RigidBody,
        name_prefix: str,
        attachment_offset: Vector3,
        orientation: Quaternion,
    ) -> None:
        if module.type == module.Type.CORE:
            self._make_core(
                cast(Core, module), body, name_prefix, attachment_offset, orientation
            )
        elif module.type == module.Type.BRICK:
            self._make_brick(
                cast(Brick, module), body, name_prefix, attachment_offset, orientation
            )
        elif module.type == module.type.ACTIVE_HINGE:
            self._make_active_hinge(
                cast(ActiveHinge, module),
                body,
                name_prefix,
                attachment_offset,
                orientation,
            )
        else:
            raise NotImplementedError("Module type not implemented")

    def _make_core(
        self,
        module: Core,
        body: RigidBody,
        name_prefix: str,
        attachment_point: Vector3,
        orientation: Quaternion,
    ) -> None:
        BOUNDING_BOX = Vector3([0.089, 0.089, 0.0603])  # meter
        MASS = 0.250  # kg
        SLOT_OFFSET = 0.089 / 2.0  # meter

        # attachment position is always at center of core
        position = attachment_point

        body.collisions.append(
            Collision(
                f"{name_prefix}_core_collision",
                position,
                orientation,
                MASS,
                BOUNDING_BOX,
            )
        )
        body.visuals.append(
            Visual(
                f"{name_prefix}_core_visual",
                position,
                orientation,
                "model://rg_robot/meshes/CoreComponent.dae",
                (1.0, 1.0, 0.0, 1.0),
            )
        )

        for (name_suffix, slot, angle) in [
            ("front", module.front, 0.0),
            ("back", module.back, math.pi),
            ("left", module.left, math.pi / 2.0),
            ("right", module.right, math.pi / 2.0 * 3),
        ]:
            if slot is not None:
                rotation = (
                    orientation
                    * Quaternion.from_eulers([0.0, 0.0, angle])
                    * Quaternion.from_eulers([slot.rotation, 0, 0])
                )

                self._make_module(
                    slot.module,
                    body,
                    f"{name_prefix}_{name_suffix}",
                    position + rotation * Vector3([SLOT_OFFSET, 0.0, 0.0]),
                    rotation,
                )

    def _make_brick(
        self,
        module: Brick,
        body: RigidBody,
        name_prefix: str,
        attachment_point: Vector3,
        orientation: Quaternion,
    ) -> None:
        BOUNDING_BOX = Vector3([0.06288625, 0.06288625, 0.0603])  # meter
        MASS = 0.030  # kg
        SLOT_OFFSET = 0.06288625 / 2.0  # meter

        position = attachment_point + orientation * Vector3(
            [BOUNDING_BOX[0] / 2.0, 0.0, 0.0]
        )

        body.collisions.append(
            Collision(
                f"{name_prefix}_brick_collision",
                position,
                orientation,
                MASS,
                BOUNDING_BOX,
            )
        )
        body.visuals.append(
            Visual(
                f"{name_prefix}_brick_visual",
                position,
                orientation,
                "model://rg_robot/meshes/FixedBrick.dae",
                (1.0, 0.0, 0.0, 1.0),
            )
        )

        for (name_suffix, slot, angle) in [
            ("front", module.front, 0.0),
            ("back", module.back, math.pi),
            ("left", module.left, math.pi / 2.0),
            ("right", module.right, math.pi / 2.0 * 3),
        ]:
            if slot is not None:
                rotation = (
                    orientation
                    * Quaternion.from_eulers([0.0, 0.0, angle])
                    * Quaternion.from_eulers([slot.rotation, 0, 0])
                )

                self._make_module(
                    slot.module,
                    body,
                    f"{name_prefix}_{name_suffix}",
                    position + rotation * Vector3([SLOT_OFFSET, 0.0, 0.0]),
                    rotation,
                )

    def _make_active_hinge(
        self,
        module: ActiveHinge,
        body: RigidBody,
        name_prefix: str,
        attachment_point: Vector3,
        orientation: Quaternion,
    ) -> None:
        FRAME_BOUNDING_BOX = Vector3([0.04525, 0.053, 0.0165891])  # meter
        SERVO1_BOUNDING_BOX = Vector3([0.0583, 0.0512, 0.020])  # meter
        SERVO2_BOUNDING_BOX = Vector3([0.002, 0.053, 0.053])  # meter

        FRAME_MASS = 0.011  # kg
        SERVO1_MASS = 0.058  # kg
        SERVO2_MASS = 0.0  # kg. we simplify by only using the weight of the first box

        SERVO_OFFSET = 0.0299  # meter. distance from frame to servo
        JOINT_OFFSET = 0.0119  # meter. distance from frame to joint

        ATTACHMENT_OFFSET = SERVO1_BOUNDING_BOX[0] / 2.0 + SERVO2_BOUNDING_BOX[0]

        frame_position = attachment_point + orientation * Vector3(
            [FRAME_BOUNDING_BOX[0] / 2.0, 0.0, 0.0]
        )
        servo_body_position = body.position + body.orientation * (
            frame_position + orientation * Vector3([SERVO_OFFSET, 0.0, 0.0])
        )
        servo_body_orientation = body.orientation * orientation
        joint_position = body.position + body.orientation * (
            frame_position + orientation * Vector3([JOINT_OFFSET, 0.0, 0.0])
        )
        joint_orientation = body.orientation * orientation

        body.collisions.append(
            Collision(
                f"{name_prefix}_activehingeframe_collision",
                frame_position,
                orientation,
                FRAME_MASS,
                FRAME_BOUNDING_BOX,
            )
        )
        body.visuals.append(
            Visual(
                f"{name_prefix}_activehingeframe_visual",
                frame_position,
                orientation,
                "model://rg_robot/meshes/ActiveHinge_Frame.dae",
                (0.0, 1.0, 0.0, 1.0),
            )
        )

        next_body = RigidBody(
            f"{name_prefix}_activehinge",
            servo_body_position,
            servo_body_orientation,
        )
        self.robot.bodies.append(next_body)
        self.robot.joints.append(
            Joint(
                f"{name_prefix}_activehinge",
                body,
                next_body,
                joint_position,
                joint_orientation,
                Vector3([0.0, 1.0, 0.0]),
            )
        )

        next_body.collisions.append(
            Collision(
                f"{name_prefix}_activehingemotor_collision",
                Vector3(),
                Quaternion(),
                SERVO1_MASS,
                SERVO1_BOUNDING_BOX,
            )
        )
        next_body.visuals.append(
            Visual(
                f"{name_prefix}_activehingemotor_visual",
                Vector3(),
                Quaternion(),
                "model://rg_robot/meshes/ActiveHinge_Frame.dae",
                (0.0, 1.0, 0.0, 1.0),
            )
        )

        if module.attachment is not None:
            rotation = Quaternion.from_eulers([module.attachment.rotation, 0.0, 0.0])

            self._make_module(
                module.attachment.module,
                next_body,
                f"{name_prefix}_attachment",
                rotation * Vector3([ATTACHMENT_OFFSET, 0.0, 0.0]),
                rotation,
            )
