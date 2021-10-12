import math
from typing import cast

from pyrr import Quaternion, Vector3
from revolve2.core.physics_robot.joint import Joint
from revolve2.core.physics_robot.rigid_body import RigidBody
from revolve2.core.physics_robot.rigid_part import RigidPart

from ..physics_robot import RigidBody, Robot
from .active_hinge import ActiveHinge
from .brick import Brick
from .core import Core
from .modular_robot import ModularRobot
from .module import Module


def to_physics_robot(modular_robot: ModularRobot) -> Robot:
    return _PhysicsRobotBuilder().build(modular_robot)


class _PhysicsRobotBuilder:
    robot: Robot

    def build(self, modular_robot: ModularRobot) -> Robot:
        self.robot = Robot([], [])

        core_body = RigidBody()
        self.robot.bodies.append(core_body)

        self._make_module(
            modular_robot.body.core, core_body, "origin", Vector3(), Quaternion()
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

        position = attachment_offset + orientation * Vector3([sizexy / 2.0, 0.0, 0.0])

        body.parts.append(
            RigidPart(
                position, orientation, mass, "core_visual_todo", "core_collision_todo"
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
                position, orientation, mass, "brick_visual_todo", "brick_collision_todo"
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
        sizex = 0.022
        sizey = 0.03575
        sizez = 0.01
        mass = 1.0

        attachment_sizex = 0.026
        attachment_sizey = 0.02575
        attachment_sizez = 0.015

        position = attachment_offset + orientation * Vector3([sizex / 2.0, 0.0, 0.0])
        position_attachment = attachment_offset + orientation * Vector3(
            [sizex / 2.0 + attachment_sizex / 2.0, 0.0, 0.0]
        )
        middle = orientation * Vector3(
            [-attachment_sizex / 2.0, 0.0, 0.0]  # TODO offset is incorrect
        )  # attachment_offset + orientation * Vector3([sizex / 2.0, 0.0, 0.0])

        body.parts.append(
            RigidPart(
                position,
                orientation,
                mass,
                "active_hinge_visual_todo",
                "active_hinge_collision_todo",
            )
        )

        next_body = RigidBody()
        self.robot.bodies.append(next_body)
        self.robot.joints.append(Joint(body, next_body))

        body.parts.append(
            RigidPart(
                Vector3(),
                Quaternion(),
                mass,
                "active_hinge_attachment_visual_todo",
                "active_hinge_attachment_collision_todo",
            )
        )

        if module.attachment is not None:
            self._make_module_directed(
                module.attachment.module,
                body,
                f"{name_prefix}_attachment",
                Vector3(),
                Quaternion(),
                attachment_sizex / 2.0,
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
