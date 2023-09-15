from ._brick import Brick
from ._core import Core
from ._module import Module
from ._active_hinge import ActiveHinge
from revolve2.simulation.actor import Actor, RigidBody
from pyrr import Quaternion, Vector3
from typing import Any

from abc import ABC, abstractmethod
class ActorBuilder(ABC):
    _STATIC_FRICTION = 1.0
    _DYNAMIC_FRICTION = 1.0

    robot: Actor
    dof_ids: list[int]

    @abstractmethod
    def build(self, body: Any) -> tuple[Actor, list[int]]:
        """
        Build a body into an Actor.

        :param body: the body.
        :return: actor, dof_ids.
        """
        pass

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

    @abstractmethod
    def _make_core(
        self,
        module: Core,
        body: RigidBody,
        name_prefix: str,
        attachment_point: Vector3,
        orientation: Quaternion,
    ) -> None:
        """
        Make the Core.

        :param module: The Core.
        :param body: The Body.
        :param name_prefix:  Name Prefix.
        :param attachment_point: The attachment point.
        :param orientation: The orientation.
        """
        pass

    @abstractmethod
    def _make_brick(
        self,
        module: Brick,
        body: RigidBody,
        name_prefix: str,
        attachment_point: Vector3,
        orientation: Quaternion,
    ) -> None:
        """
        Make a Brick.

        :param module: The Brick.
        :param body: The Body.
        :param name_prefix:  Name Prefix.
        :param attachment_point: The attachment point.
        :param orientation: The orientation.
        """
        pass

    @abstractmethod
    def _make_active_hinge(
        self,
        module: ActiveHinge,
        body: RigidBody,
        name_prefix: str,
        attachment_point: Vector3,
        orientation: Quaternion,
    ) -> None:
        """
        Make a Hinge.

        :param module: The Hinge.
        :param body: The Body.
        :param name_prefix:  Name Prefix.
        :param attachment_point: The attachment point.
        :param orientation: The orientation.
        """
        pass

