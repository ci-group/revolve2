import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from pyrr import Quaternion, Vector3
from revolve2.core.physics.actor import Actor, Collision, Joint, RigidBody, Visual

from ._active_hinge import ActiveHinge
from ._brick import Brick
from ._core import Core
from ._module import Module
from ._not_finalized_error import NotFinalizedError


class Body:
    """Body of a modular robot."""

    core: Core
    _is_finalized: bool

    def __init__(self) -> None:
        """Initialize this object."""
        self.core = Core(0.0)
        self._is_finalized = False

    def finalize(self) -> None:
        """
        Finalize the body by assigning ids to all modules.

        :raises RuntimeError: In case this body has already been finalized before.
        """
        if self._is_finalized:
            raise RuntimeError("Cannot finalize twice.")
        assigner = _Finalizer(self)
        assigner.finalize()
        self._is_finalized = True

    @property
    def is_finalized(self) -> bool:
        """
        Check if the robot has been finalized.

        :returns: Wether the robot has been finalized.
        """
        return self._is_finalized

    def to_actor(self) -> Tuple[Actor, List[int]]:
        """
        Create an actor from this body.

        :returns: (the actor, ids of modules matching the joints in the actor)
        :raises NotFinalizedError: In case this body has not yet been finalized.
        """
        if not self.is_finalized:
            raise NotFinalizedError()
        return _ActorBuilder().build(self)

    def find_active_hinges(self) -> List[ActiveHinge]:
        """
        Find all active hinges in the body.

        :returns: A list of all active hinges in the body
        :raises NotFinalizedError: In case this body has not yet been finalized.
        """
        if not self.is_finalized:
            raise NotFinalizedError()
        return _ActiveHingeFinder().find(self)

    def find_bricks(self) -> List[Brick]:
        """
        Find all bricks in the body.

        :returns: A list of all bricks in the body
        :raises NotFinalizedError: In case this body has not yet been finalized.
        """
        if not self.is_finalized:
            raise NotFinalizedError()
        return _BrickFinder().find(self)

    def grid_position(self, module: Module) -> Vector3:
        """
        Calculate the position of this module in a 3d grid with the core as center.

        The distance between all modules is assumed to be one grid cell.
        All module angles must be multiples of 90 degrees.

        :param module: The module to calculate the position for.
        :returns: The calculated position.
        :raises NotImplementedError: In case a module is encountered that is not supported.
        """
        # TODO make this into a function that maps the complete robot to a grid

        position = Vector3()

        parent = module._parent
        child_index = module._parent_child_index
        while parent is not None and child_index is not None:
            child = parent.children[child_index]
            assert child is not None
            assert np.isclose(child.rotation % (math.pi / 2.0), 0.0)

            position = Quaternion.from_eulers((child.rotation, 0.0, 0.0)) * position
            position += Vector3([1, 0, 0])
            rotation: Quaternion
            if isinstance(parent, Core):
                if child_index == Core.FRONT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, 0.0))
                elif child_index == Core.LEFT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 1))
                elif child_index == Core.BACK:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 2))
                elif child_index == Core.RIGHT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 3))
                else:
                    raise NotImplementedError()
            elif isinstance(parent, Brick):
                if child_index == Brick.FRONT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, 0.0))
                elif child_index == Brick.LEFT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 1))
                elif child_index == Brick.RIGHT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 3))
                else:
                    raise NotImplementedError()
            elif isinstance(parent, ActiveHinge):
                if child_index == ActiveHinge.ATTACHMENT:
                    rotation = Quaternion()
                else:
                    raise NotImplementedError()
            else:
                raise NotImplementedError()
            position = rotation * position
            position = Vector3.round(position)

            child_index = parent._parent_child_index
            parent = parent._parent

        return position

    def to_grid(
        self,
    ) -> Tuple[List[List[List[Optional[Module]]]], Tuple[int, int, int]]:
        """
        Convert the tree structure to a grid.

        The distance between all modules is assumed to be one grid cell.
        All module angles must be multiples of 90 degrees.

        The grid is indexed depth, width, height, or x, y, z, from the perspective of the core.

        :returns: The created grid with cells set to either a Module or None and a tuple representing the position of the core.
        :raises NotImplementedError: In case a module is encountered that is not supported.

        # noqa: DAR402 NotImplementedError
        """
        return _GridMaker().make_grid(self)


class _Finalizer:
    _body: Body
    _next_id: int

    def __init__(self, body: Body) -> None:
        self._body = body
        self._next_id = 0

    def finalize(self) -> None:
        self._finalize_recur(self._body.core)

    def _finalize_recur(self, module: Module) -> None:
        module.id = self._next_id
        self._next_id += 1
        for i, child in enumerate(module.children):
            if child is not None:
                child._parent = module
                child._parent_child_index = i
                self._finalize_recur(child)


class _GridMaker:
    @dataclass
    class _Cell:
        x: int
        y: int
        z: int
        module: Module

    _core_pos: Tuple[int, int, int]
    _cells: List[_Cell]

    def __init__(self) -> None:
        self._cells = []

    def make_grid(
        self, body: Body
    ) -> Tuple[List[List[List[Optional[Module]]]], Tuple[int, int, int]]:
        self._make_grid_recur(body.core, Vector3(), Quaternion())

        minx = min([cell.x for cell in self._cells])
        maxx = max([cell.x for cell in self._cells])
        miny = min([cell.y for cell in self._cells])
        maxy = max([cell.y for cell in self._cells])
        minz = min([cell.z for cell in self._cells])
        maxz = max([cell.z for cell in self._cells])

        depth = maxx - minx + 1
        width = maxy - miny + 1
        height = maxz - minz + 1

        grid: List[List[List[Optional[Module]]]] = []
        for _ in range(depth):
            y: List[List[Optional[Module]]] = []
            for _ in range(width):
                y.append([None] * (height))
            grid.append(y)

        for cell in self._cells:
            grid[cell.x - minx][cell.y - miny][cell.z - minz] = cell.module

        return grid, (-minx, -miny, -minz)

    def _make_grid_recur(
        self, module: Module, position: Vector3, orientation: Quaternion
    ) -> None:
        self._cells.append(
            self._Cell(round(position.x), round(position.y), round(position.z), module)
        )

        if isinstance(module, Core):
            for (child_index, angle) in [
                (Core.FRONT, 0.0),
                (Core.BACK, math.pi),
                (Core.LEFT, math.pi / 2.0),
                (Core.RIGHT, math.pi / 2.0 * 3),
            ]:
                child = module.children[child_index]

                if child is not None:
                    assert np.isclose(child.rotation % (math.pi / 2.0), 0.0)

                    rotation = (
                        orientation
                        * Quaternion.from_eulers([0.0, 0.0, angle])
                        * Quaternion.from_eulers([child.rotation, 0, 0])
                    )

                    self._make_grid_recur(
                        child, position + rotation * Vector3([1.0, 0.0, 0.0]), rotation
                    )
        elif isinstance(module, Brick):
            for (child_index, angle) in [
                (Brick.FRONT, 0.0),
                (Brick.LEFT, math.pi / 2.0),
                (Brick.RIGHT, math.pi / 2.0 * 3),
            ]:
                child = module.children[child_index]

                if child is not None:
                    assert np.isclose(child.rotation % (math.pi / 2.0), 0.0)

                    rotation = (
                        orientation
                        * Quaternion.from_eulers([0.0, 0.0, angle])
                        * Quaternion.from_eulers([child.rotation, 0, 0])
                    )

                    self._make_grid_recur(
                        child, position + rotation * Vector3([1.0, 0.0, 0.0]), rotation
                    )
        elif isinstance(module, ActiveHinge):
            child = module.children[ActiveHinge.ATTACHMENT]

            if child is not None:
                assert np.isclose(child.rotation % (math.pi / 2.0), 0.0)

                rotation = Quaternion.from_eulers([child.rotation, 0.0, 0.0])

                self._make_grid_recur(
                    child, position + rotation * Vector3([1.0, 0.0, 0.0]), rotation
                )
        else:
            raise NotImplementedError()


class _ActorBuilder:
    _STATIC_FRICTION = 1.0
    _DYNAMIC_FRICTION = 1.0

    robot: Actor
    dof_ids: List[int]

    def build(self, body: Body) -> Tuple[Actor, List[int]]:
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

        self._make_module(body.core, origin_body, "origin", Vector3(), Quaternion())

        return (self.robot, self.dof_ids)

    def _make_module(
        self,
        module: Module,
        body: RigidBody,
        name_prefix: str,
        attachment_offset: Vector3,
        orientation: Quaternion,
    ) -> None:
        if isinstance(module, Core):
            self._make_core(
                module,
                body,
                name_prefix,
                attachment_offset,
                orientation,
            )
        elif isinstance(module, Brick):
            self._make_brick(
                module,
                body,
                name_prefix,
                attachment_offset,
                orientation,
            )
        elif isinstance(module, ActiveHinge):
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
        module: Core,
        body: RigidBody,
        name_prefix: str,
        attachment_point: Vector3,
        orientation: Quaternion,
    ) -> None:
        BOUNDING_BOX = Vector3([0.089, 0.089, 0.0603])  # meter
        MASS = 0.250  # kg
        CHILD_OFFSET = 0.089 / 2.0  # meter

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

                self._make_module(
                    child,
                    body,
                    f"{name_prefix}_{name_suffix}",
                    position + rotation * Vector3([CHILD_OFFSET, 0.0, 0.0]),
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
        CHILD_OFFSET = 0.06288625 / 2.0  # meter

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

                self._make_module(
                    child,
                    body,
                    f"{name_prefix}_{name_suffix}",
                    position + rotation * Vector3([CHILD_OFFSET, 0.0, 0.0]),
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
        # this function used to be good but is now a hacky way to create a body that has no self clipping
        # TODO fix

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

        frame_position = attachment_point + orientation * Vector3(
            [FRAME_OFFSET / 2.0, 0.0, 0.0]
        )
        frame_position_real = attachment_point + orientation * Vector3(
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
                frame_position_real,
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
                range=module.RANGE,
                effort=module.EFFORT,
                velocity=module.VELOCITY,
            )
        )
        self.dof_ids.append(module.id)

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

        child = module.children[ActiveHinge.ATTACHMENT]
        if child is not None:
            rotation = Quaternion.from_eulers([child.rotation, 0.0, 0.0])

            self._make_module(
                child,
                next_body,
                f"{name_prefix}_attachment",
                rotation * Vector3([ATTACHMENT_OFFSET, 0.0, 0.0]),
                rotation,
            )


class _ActiveHingeFinder:
    _active_hinges: List[ActiveHinge]

    def __init__(self) -> None:
        self._active_hinges = []

    def find(self, body: Body) -> List[ActiveHinge]:
        self._find_recur(body.core)
        return self._active_hinges

    def _find_recur(self, module: Module) -> None:
        if isinstance(module, ActiveHinge):
            self._active_hinges.append(module)
        for child in module.children:
            if child is not None:
                self._find_recur(child)


class _BrickFinder:
    _bricks: List[Brick]

    def __init__(self) -> None:
        self._bricks = []

    def find(self, body: Body) -> List[Brick]:
        self._find_recur(body.core)
        return self._bricks

    def _find_recur(self, module: Module) -> None:
        if isinstance(module, Brick):
            self._bricks.append(module)
        for child in module.children:
            if child is not None:
                self._find_recur(child)
