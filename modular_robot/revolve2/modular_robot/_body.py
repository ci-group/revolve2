import math
from dataclasses import dataclass

import numpy as np
from pyrr import Quaternion, Vector3
from revolve2.modular_robot._active_hinge import ActiveHinge
from revolve2.modular_robot._body_state import BodyState
from revolve2.modular_robot._brick import Brick
from revolve2.modular_robot._core import Core
from revolve2.modular_robot._module import Module
from revolve2.modular_robot._not_finalized_error import NotFinalizedError
from revolve2.modular_robot._properties import PropertySet
from revolve2.simulation.actor import Actor, Collision, Joint, RigidBody
from revolve2.simulation.running._results import ActorState


class Body:
    """Body of a modular robot."""

    core: Core
    _is_finalized: bool
    property_set: PropertySet

    def __init__(self, property_set: PropertySet) -> None:
        """Initialize this object.

        :param property_set: The robots properties.
        """
        self.property_set = property_set
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

    def to_actor(self) -> tuple[Actor, list[int]]:
        """
        Create an actor from this body.

        :returns: (the actor, ids of modules matching the joints in the actor)
        :raises NotFinalizedError: In case this body has not yet been finalized.
        """
        if not self.is_finalized:
            raise NotFinalizedError()
        return _ActorBuilder().build(self)

    def find_active_hinges(self) -> list[ActiveHinge]:
        """
        Find all active hinges in the body.

        :returns: A list of all active hinges in the body
        :raises NotFinalizedError: In case this body has not yet been finalized.
        """
        if not self.is_finalized:
            raise NotFinalizedError()
        return _ActiveHingeFinder().find(self)

    def find_bricks(self) -> list[Brick]:
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
    ) -> tuple[list[list[list[Module | None]]], tuple[int, int, int]]:
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

    def body_state_from_actor_state(self, actor_state: ActorState) -> BodyState:
        """
        Transform an actor state (part of a simulation result) to modular robot body space.

        :param actor_state: The actor state to convert from.
        :returns: The transformed actor state.
        """
        return BodyState(
            core_position=actor_state.position, core_orientation=actor_state.orientation
        )


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

    _core_pos: tuple[int, int, int]
    _cells: list[_Cell]

    def __init__(self) -> None:
        self._cells = []

    def make_grid(
        self, body: Body
    ) -> tuple[list[list[list[Module | None]]], tuple[int, int, int]]:
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

        grid: list[list[list[Module | None]]] = []
        for _ in range(depth):
            y: list[list[Module | None]] = []
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
            for child_index, angle in [
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
            for child_index, angle in [
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

    property_set: PropertySet
    robot: Actor
    dof_ids: list[int]

    def build(self, body: Body) -> tuple[Actor, list[int]]:
        self.robot = Actor([], [])
        self.property_set = body.property_set
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
        def __get_attachment_offset(attachment_position: int, angle: float) -> Vector3:
            """
            Calculate offset.

            :param attachment_position: index position of attachment on core.
            :param angle: angle of core side for determining offset axis / direction.
            :returns: The offset Vector.
            """
            HO, VO = core_properties.HORIZONTAL_OFFSET, core_properties.VERTICAL_OFFSET
            h_cond, v_cond = attachment_position % 3, attachment_position / 3

            h_offset, v_offset = 0.0, 0.0
            if h_cond == 1:
                h_offset = -HO
            elif h_cond == 0:
                h_offset = HO
            h_offset = -h_offset if angle >= math.pi else h_offset
            if v_cond <= 1:
                v_offset = VO
            elif v_cond > 2:
                v_offset = -VO

            offset = (
                Vector3([0.0, h_offset, v_offset])
                if angle % math.pi == 0
                else Vector3([-h_offset, 0.0, v_offset])
            )
            return offset

        core_properties = self.property_set.get_core_properties()
        BOUNDING_BOX = core_properties.BOUNDING_BOX  # meter
        MASS = core_properties.MASS  # kg
        CHILD_OFFSET = core_properties.CHILD_OFFSET  # meter

        # attachment position is always at center of core
        position = attachment_point

        body.collisions.append(
            Collision(
                name=f"{name_prefix}_core_collision",
                position=position,
                orientation=orientation,
                mass=MASS,
                bounding_box=BOUNDING_BOX,
                color=core_properties.color,
            )
        )

        for name_suffix, child_index, angle in [
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

                attachment_offset = (
                    __get_attachment_offset(child.attachment_position, angle)
                    if child.attachment_position
                    else Vector3([0.0, 0.0, 0.0])
                )

                self._make_module(
                    child,
                    body,
                    f"{name_prefix}_{name_suffix}",
                    position
                    + rotation * Vector3([CHILD_OFFSET, 0.0, 0.0])
                    + attachment_offset,
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

        brick_properties = self.property_set.get_brick_properties()
        BOUNDING_BOX = brick_properties.BOUNDING_BOX  # meter
        MASS = brick_properties.MASS  # kg
        CHILD_OFFSET = brick_properties.CHILD_OFFSET  # meter

        position = attachment_point + orientation * Vector3(
            [BOUNDING_BOX[0] / 2.0, 0.0, 0.0]
        )

        body.collisions.append(
            Collision(
                name=f"{name_prefix}_brick_collision",
                position=position,
                orientation=orientation,
                mass=MASS,
                bounding_box=BOUNDING_BOX,
                color=brick_properties.color,
            )
        )

        for name_suffix, child_index, angle in [
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
        active_hinge_properties = self.property_set.get_active_hinge_properties()

        FRAME_BOUNDING_BOX = active_hinge_properties.FRAME_BOUNDING_BOX  # meter
        FRAME_OFFSET = active_hinge_properties.FRAME_OFFSET
        SERVO1_BOUNDING_BOX = active_hinge_properties.SERVO1_BOUNDING_BOX  # meter
        SERVO2_BOUNDING_BOX = active_hinge_properties.SERVO2_BOUNDING_BOX  # meter

        FRAME_MASS = active_hinge_properties.FRAME_MASS  # kg
        SERVO1_MASS = active_hinge_properties.SERVO1_MASS  # kg
        SERVO2_MASS = (
            active_hinge_properties.SERVO2_MASS
        )  # kg. we simplify by only using the weight of the first box

        SERVO_OFFSET = (
            active_hinge_properties.SERVO_OFFSET
        )  # meter. distance from frame to servo
        JOINT_OFFSET = (
            active_hinge_properties.JOINT_OFFSET
        )  # meter. distance from frame to joint

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
                name=f"{name_prefix}_activehingeframe_collision",
                position=frame_position_real,
                orientation=orientation,
                mass=FRAME_MASS,
                bounding_box=FRAME_BOUNDING_BOX,
                color=active_hinge_properties.color,
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
                range=active_hinge_properties.RANGE,
                effort=active_hinge_properties.EFFORT,
                velocity=active_hinge_properties.VELOCITY,
            )
        )
        self.dof_ids.append(module.id)

        next_body.collisions.append(
            Collision(
                name=f"{name_prefix}_activehingemotor_collision1",
                position=Vector3(),
                orientation=Quaternion(),
                mass=SERVO1_MASS,
                bounding_box=SERVO1_BOUNDING_BOX,
                color=active_hinge_properties.color,
            )
        )
        next_body.collisions.append(
            Collision(
                name=f"{name_prefix}_activehingemotor_collision2",
                position=SERVO_BBOX2_POSITION,
                orientation=Quaternion(),
                mass=SERVO2_MASS,
                bounding_box=SERVO2_BOUNDING_BOX,
                color=active_hinge_properties.color,
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
    _active_hinges: list[ActiveHinge]

    def __init__(self) -> None:
        self._active_hinges = []

    def find(self, body: Body) -> list[ActiveHinge]:
        self._find_recur(body.core)
        return self._active_hinges

    def _find_recur(self, module: Module) -> None:
        if isinstance(module, ActiveHinge):
            self._active_hinges.append(module)
        for child in module.children:
            if child is not None:
                self._find_recur(child)


class _BrickFinder:
    _bricks: list[Brick]

    def __init__(self) -> None:
        self._bricks = []

    def find(self, body: Body) -> list[Brick]:
        self._find_recur(body.core)
        return self._bricks

    def _find_recur(self, module: Module) -> None:
        if isinstance(module, Brick):
            self._bricks.append(module)
        for child in module.children:
            if child is not None:
                self._find_recur(child)
