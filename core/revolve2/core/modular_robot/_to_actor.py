import math
from typing import List, Tuple

from pyrr import Quaternion, Vector3

from revolve2.core.physics.actor import Actor, Collision, Joint, RigidBody, Visual

from ._active_hinge import ActiveHinge
from ._analyzer import Analyzer
from ._analyzer_module import AnalyzerModule
from ._brick import Brick
from ._core import Core
from ._module import Module


def to_actor(analyzer: Analyzer) -> Tuple[Actor, List[AnalyzerModule]]:
    """
    Create actor from analyzer

    :returns: (the actor, analyzer modules matching the joints in the actor)
    """
    return _ActorBuilder().build(analyzer)


class _ActorBuilder:
    _STATIC_FRICTION = 0.8
    _DYNAMIC_FRICTION = 0.5

    robot: Actor
    dof_ids: List[AnalyzerModule]

    def build(self, analyzer: Analyzer) -> Tuple[Actor, List[AnalyzerModule]]:
        self.robot = Actor([], [])
        self.dof_ids = []

        origin_body = RigidBody(
            "origin",
            Vector3(),
            Quaternion(),
            self._STATIC_FRICTION,
            self._DYNAMIC_FRICTION,
        )
        self.robot.bodies.append(origin_body)

        self._make_module(analyzer.core, origin_body, "origin", Vector3(), Quaternion())

        return (self.robot, self.dof_ids)

    def _make_module(
        self,
        module: AnalyzerModule,
        body: RigidBody,
        name_prefix: str,
        attachment_offset: Vector3,
        orientation: Quaternion,
    ) -> None:
        if module.type == Module.Type.CORE:
            self._make_core(
                module,
                body,
                name_prefix,
                attachment_offset,
                orientation,
            )
        elif module.type == Module.Type.BRICK:
            self._make_brick(
                module,
                body,
                name_prefix,
                attachment_offset,
                orientation,
            )
        elif module.type == Module.Type.ACTIVE_HINGE:
            self._make_active_hinge(
                module,
                body,
                name_prefix,
                attachment_offset,
                orientation,
            )
        else:
            raise NotImplementedError("Module type not implemented")

    def _make_core(
        self,
        analyzer_module: AnalyzerModule,
        body: RigidBody,
        name_prefix: str,
        attachment_point: Vector3,
        orientation: Quaternion,
    ) -> None:
        BOUNDING_BOX = Vector3([0.089, 0.089, 0.0603])  # meter
        MASS = 0.250  # kg
        CHILD_OFFSET = 0.089 / 2.0  # meter

        module = analyzer_module.module
        assert isinstance(module, Core)

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
                (1.0, 1.0, 0.0),
            )
        )

        for (name_suffix, child_index, angle) in [
            ("front", Core.FRONT, 0.0),
            ("back", Core.BACK, math.pi),
            ("left", Core.LEFT, math.pi / 2.0),
            ("right", Core.RIGHT, math.pi / 2.0 * 3),
        ]:
            child = module.children[child_index]
            if child is not None:
                rotation = (
                    orientation
                    * Quaternion.from_eulers([0.0, 0.0, angle])
                    * Quaternion.from_eulers([child.rotation, 0, 0])
                )

                analyzer_child = analyzer_module.children[child_index]
                assert analyzer_child is not None

                self._make_module(
                    analyzer_child,
                    body,
                    f"{name_prefix}_{name_suffix}",
                    position + rotation * Vector3([CHILD_OFFSET, 0.0, 0.0]),
                    rotation,
                )

    def _make_brick(
        self,
        analyzer_module: AnalyzerModule,
        body: RigidBody,
        name_prefix: str,
        attachment_point: Vector3,
        orientation: Quaternion,
    ) -> None:
        BOUNDING_BOX = Vector3([0.06288625, 0.06288625, 0.0603])  # meter
        MASS = 0.030  # kg
        CHILD_OFFSET = 0.06288625 / 2.0  # meter

        module = analyzer_module.module
        assert isinstance(module, Brick)

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
                (1.0, 0.0, 0.0),
            )
        )

        for (name_suffix, child_index, angle) in [
            ("front", Brick.FRONT, 0.0),
            ("left", Brick.LEFT, math.pi / 2.0),
            ("right", Brick.RIGHT, math.pi / 2.0 * 3),
        ]:
            child = module.children[child_index]
            if child is not None:
                rotation = (
                    orientation
                    * Quaternion.from_eulers([0.0, 0.0, angle])
                    * Quaternion.from_eulers([child.rotation, 0, 0])
                )

                analyzer_child = analyzer_module.children[child_index]
                assert analyzer_child is not None

                self._make_module(
                    analyzer_child,
                    body,
                    f"{name_prefix}_{name_suffix}",
                    position + rotation * Vector3([CHILD_OFFSET, 0.0, 0.0]),
                    rotation,
                )

    def _make_active_hinge(
        self,
        analyzer_module: AnalyzerModule,
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

        SERVO_BBOX2_POSITION = Vector3(
            [SERVO1_BOUNDING_BOX[0] / 2.0 + SERVO2_BOUNDING_BOX[0] / 2.0, 0.0, 0.0]
        )

        ATTACHMENT_OFFSET = SERVO1_BOUNDING_BOX[0] / 2.0 + SERVO2_BOUNDING_BOX[0]

        module = analyzer_module.module
        assert isinstance(module, ActiveHinge)

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
                (0.0, 1.0, 0.0),
            )
        )

        next_body = RigidBody(
            f"{name_prefix}_activehinge",
            servo_body_position,
            servo_body_orientation,
            self._STATIC_FRICTION,
            self._DYNAMIC_FRICTION,
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
        self.dof_ids.append(analyzer_module)

        next_body.collisions.append(
            Collision(
                f"{name_prefix}_activehingemotor_collision1",
                Vector3(),
                Quaternion(),
                SERVO1_MASS,
                SERVO1_BOUNDING_BOX,
            )
        )
        next_body.collisions.append(
            Collision(
                f"{name_prefix}_activehingemotor_collision2",
                SERVO_BBOX2_POSITION,
                Quaternion(),
                SERVO2_MASS,
                SERVO2_BOUNDING_BOX,
            )
        )
        next_body.visuals.append(
            Visual(
                f"{name_prefix}_activehingemotor_visual",
                Vector3(),
                Quaternion(),
                "model://rg_robot/meshes/ActiveCardanHinge_Servo_Holder.dae",
                (0.0, 1.0, 0.0),
            )
        )

        child = module.children[ActiveHinge.ATTACHMENT_INDEX]
        if child is not None:
            rotation = Quaternion.from_eulers([child.rotation, 0.0, 0.0])

            analyzer_child = analyzer_module.children[ActiveHinge.ATTACHMENT_INDEX]
            assert analyzer_child is not None

            self._make_module(
                analyzer_child,
                next_body,
                f"{name_prefix}_attachment",
                rotation * Vector3([ATTACHMENT_OFFSET, 0.0, 0.0]),
                rotation,
            )
