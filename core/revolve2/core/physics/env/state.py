from __future__ import annotations

from dataclasses import dataclass
from typing import List

from pyrr import Quaternion, Vector3
from revolve2.core.database.serialize import Serializable
from revolve2.core.database.view import AnyView


@dataclass
class ActorState:
    position: Vector3
    orientation: Quaternion

    def to_database(self, db_view: AnyView) -> None:
        root = db_view.dict
        root.clear()
        root.insert(
            "position"
        ).string = f"{self.position.x} {self.position.y} {self.position.z}"
        root.insert(
            "orientation"
        ).string = f"{self.orientation.x} {self.orientation.y} {self.orientation.z} {self.orientation.w}"

    @classmethod
    def from_database(cls, db_view: AnyView) -> State:
        raise NotImplementedError()


@dataclass
class EnvironmentState:
    actor_states: List[ActorState]

    def to_database(self, db_view: AnyView) -> None:
        root = db_view.list
        root.clear()
        for state in self.actor_states:
            state.to_database(root.append())

    @classmethod
    def from_database(cls, db_view: AnyView) -> State:
        raise NotImplementedError()


@dataclass
class State:
    envs: List[EnvironmentState]

    def to_database(self, db_view: AnyView) -> None:
        root = db_view.list
        root.clear()
        for env in self.envs:
            env.to_database(root.append())

    @classmethod
    def from_database(cls, db_view: AnyView) -> State:
        raise NotImplementedError()
