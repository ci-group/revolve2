import math
from typing import cast

from pyrr import Quaternion, Vector3, vector
from revolve2.core.physics_robot.joint import Joint
from revolve2.core.physics_robot.rigid_body import RigidBody
from revolve2.core.physics_robot.rigid_part import RigidPart

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
        attachment_offset: Vector3,
        orientation: Quaternion,
    ) -> None:
        sizexy = 0.089
        sizez = 0.045
        mass = 1.0

        # core is always at origin
        position = attachment_offset

        body.parts.append(
            RigidPart(
                f"{name_prefix}_core",
                position,
                orientation,
                mass,
                "model://rg_robot/meshes/CoreComponent.dae",
                (1.0, 1.0, 0.0, 1.0),
                Vector3([sizexy, sizexy, sizez]),
            )
        )

        if module.front is not None:
            self._make_module_directed(
                module.front.module,
                body,
                f"{name_prefix}_front",
                position,
                orientation,
                sizexy / 2.0,
                Quaternion.from_eulers([0.0, 0.0, 0.0])
                * Quaternion.from_eulers([module.front.rotation, 0, 0]),
            )
        if module.back is not None:
            self._make_module_directed(
                module.back.module,
                body,
                f"{name_prefix}_back",
                position,
                orientation,
                sizexy / 2.0,
                Quaternion.from_eulers([0.0, 0.0, 2 * math.pi / 2.0])
                * Quaternion.from_eulers([module.back.rotation, 0, 0]),
            )
        if module.left is not None:
            self._make_module_directed(
                module.left.module,
                body,
                f"{name_prefix}_left",
                position,
                orientation,
                sizexy / 2.0,
                Quaternion.from_eulers([0.0, 0.0, 1 * math.pi / 2.0])
                * Quaternion.from_eulers([module.left.rotation, 0, 0]),
            )
        if module.right is not None:
            self._make_module_directed(
                module.right.module,
                body,
                f"{name_prefix}_right",
                position,
                orientation,
                sizexy / 2.0,
                Quaternion.from_eulers([0.0, 0.0, 3 * math.pi / 2.0])
                * Quaternion.from_eulers([module.right.rotation, 0, 0]),
            )

    def _make_brick(
        self,
        module: Brick,
        body: RigidBody,
        name_prefix: str,
        attachment_offset: Vector3,
        orientation: Quaternion,
    ) -> None:
        sizexy = 0.041
        sizez = 0.0355
        mass = 1.0

        position = attachment_offset + orientation * Vector3([sizexy / 2.0, 0.0, 0.0])

        body.parts.append(
            RigidPart(
                f"{name_prefix}_brick",
                position,
                orientation,
                mass,
                "model://rg_robot/meshes/FixedBrick.dae",
                (1.0, 0.0, 0.0, 1.0),
                Vector3([sizexy, sizexy, sizez]),
            )
        )

        if module.front is not None:
            self._make_module_directed(
                module.front.module,
                body,
                f"{name_prefix}_front",
                position,
                orientation,
                sizexy / 2.0,
                Quaternion.from_eulers([0, 0, 0])
                * Quaternion.from_eulers([module.front.rotation, 0, 0]),
            )
        if module.back is not None:
            self._make_module_directed(
                module.back.module,
                body,
                f"{name_prefix}_back",
                position,
                orientation,
                sizexy / 2.0,
                Quaternion.from_eulers([0, 0, 2 * math.pi / 2.0])
                * Quaternion.from_eulers([module.back.rotation, 0, 0]),
            )
        if module.left is not None:
            self._make_module_directed(
                module.left.module,
                body,
                f"{name_prefix}_left",
                position,
                orientation,
                sizexy / 2.0,
                Quaternion.from_eulers([0, 0, 1 * math.pi / 2.0])
                * Quaternion.from_eulers([module.left.rotation, 0, 0]),
            )
        if module.right is not None:
            self._make_module_directed(
                module.right.module,
                body,
                f"{name_prefix}_right",
                position,
                orientation,
                sizexy / 2.0,
                Quaternion.from_eulers([0, 0, 3 * math.pi / 2.0])
                * Quaternion.from_eulers([module.right.rotation, 0, 0]),
            )

    def _make_active_hinge(
        self,
        module: ActiveHinge,
        body: RigidBody,
        name_prefix: str,
        attachment_offset: Vector3,
        orientation: Quaternion,
    ) -> None:
        frame_sizex = 0.022
        frame_sizey = 0.03575
        frame_sizez = 0.01
        frame_mass = 1.0

        motor_sizex = 0.026
        motor_sizey = 0.02575
        motor_sizez = 0.015
        motor_mass = 1.0

        new_body_offset = Vector3([frame_sizex + 0.00215, 0.0, 0.0])
        joint_offset = Vector3([-0.0085, 0.0, 0.0])

        frame_position = attachment_offset + orientation * Vector3(
            [frame_sizex / 2.0, 0.0, 0.0]
        )

        new_body_position = attachment_offset + orientation * new_body_offset

        joint_position = joint_offset

        body.parts.append(
            RigidPart(
                f"{name_prefix}_activehingeframe",
                frame_position,
                orientation,
                frame_mass,
                "model://rg_robot/meshes/ActiveHinge_Frame.dae",
                (0.0, 1.0, 0.0, 1.0),
                Vector3([frame_sizex, frame_sizey, frame_sizez]),
            )
        )

        next_body = RigidBody(
            f"{name_prefix}_activehinge", new_body_position, orientation
        )
        self.robot.bodies.append(next_body)
        self.robot.joints.append(
            Joint(
                f"{name_prefix}_activehinge",
                body,
                next_body,
                joint_position,
                orientation,
                Vector3([0.0, 1.0, 0.0]),
            )
        )

        next_body.parts.append(
            RigidPart(
                f"{name_prefix}_activehingemotor",
                Vector3(),
                Quaternion(),
                motor_mass,
                "model://rg_robot/meshes/ActiveCardanHinge_Servo_Holder.dae",
                (0.0, 1.0, 0.0, 1.0),
                Vector3([motor_sizex, motor_sizey, motor_sizez]),
            )
        )

        if module.attachment is not None:
            self._make_module_directed(
                module.attachment.module,
                next_body,
                f"{name_prefix}_attachment",
                Vector3(),
                Quaternion(),
                motor_sizex / 2.0,
                Quaternion()
                * Quaternion.from_eulers([module.attachment.rotation, 0, 0]),
            )

    def _make_module_directed(
        self,
        module: Module,
        body: RigidBody,
        name_prefix: str,
        parent_position: Vector3,
        parent_orientation: Quaternion,
        radius: float,
        normal: Quaternion,
    ) -> None:
        rotation = parent_orientation * normal

        self._make_module(
            module,
            body,
            name_prefix,
            parent_position + rotation * Vector3([radius, 0.0, 0.0]),
            rotation,
        )
