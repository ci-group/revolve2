import math
from abc import ABC, abstractmethod

import numpy as np
from pyrr import Quaternion, Vector3
from revolve2.simulation.actor import Actor, RigidBody
from revolve2.simulation.running._results import ActorState

from ._body_state import BodyState
from ._common import ActiveHinge, Brick, Core
from ._module import Module
from ._not_finalized_error import NotFinalizedError


class Body(ABC):
    """Body of a modular robot."""

    core: Core
    _is_finalized: bool

    def __init__(self) -> None:
        """Initialize this object."""
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

        robot = Actor([], [])
        dof_ids: list[int] = []

        origin_body = RigidBody(
            "origin",
            Vector3(),
            Quaternion(),
            1.0,  # STATIC_FRICTION
            1.0,  # DYNAMIC_FRICTION
        )
        robot.bodies.append(origin_body)
        self.core.build(origin_body, "origin", Vector3(), Quaternion(), robot, dof_ids)

        return (robot, dof_ids)

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
                if child_index == parent.FRONT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, 0.0))
                elif child_index == parent.LEFT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 1))
                elif child_index == parent.BACK:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 2))
                elif child_index == parent.RIGHT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 3))
                else:
                    raise NotImplementedError()
            elif isinstance(parent, Brick):
                if child_index == parent.FRONT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, 0.0))
                elif child_index == parent.LEFT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 1))
                elif child_index == parent.RIGHT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 3))
                else:
                    raise NotImplementedError()
            elif isinstance(parent, ActiveHinge):
                if child_index == parent.ATTACHMENT:
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

    def body_state_from_actor_state(self, actor_state: ActorState) -> BodyState:
        """
        Transform an actor state (part of a simulation result) to modular robot body space.

        :param actor_state: The actor state to convert from.
        :returns: The transformed actor state.
        """
        return BodyState(
            core_position=actor_state.position, core_orientation=actor_state.orientation
        )

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

    @abstractmethod
    def to_grid(self) -> tuple[list[list[list[Module | None]]], tuple[int, int, int]]:
        """
        Convert the tree structure to a grid.

        The distance between all modules is assumed to be one grid cell.
        All module angles must be multiples of 90 degrees.

        The grid is indexed depth, width, height, or x, y, z, from the perspective of the core.

        :returns: The created grid with cells set to either a Module or None and a tuple representing the position of the core.
        :raises NotImplementedError: In case a module is encountered that is not supported.

        # noqa: DAR402 NotImplementedError
        """


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
