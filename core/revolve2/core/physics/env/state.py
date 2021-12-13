from __future__ import annotations

from dataclasses import dataclass
from typing import List

from pyrr import Quaternion, Vector3
from revolve2.core.database import Data
from revolve2.core.database.serialize import Serializable


@dataclass
class ActorState(Serializable):
    position: Vector3
    orientation: Quaternion

    def serialize(self) -> Data:
        return {
            "position": f"{self.position.x} {self.position.y} {self.position.z}",
            "orientation": f"{self.orientation.x} {self.orientation.y} {self.orientation.z} {self.orientation.w}",
        }

    @classmethod
    def deserialize(cls, data: Data) -> None:
        raise NotImplementedError()


@dataclass
class EnvironmentState(Serializable):
    actor_states: List[ActorState]

    def serialize(self) -> Data:
        return [state.serialize() for state in self.actor_states]

    @classmethod
    def deserialize(cls, data: Data) -> None:
        raise NotImplementedError()


@dataclass
class State(Serializable):
    envs: List[EnvironmentState]

    def serialize(self) -> Data:
        return [env.serialize() for env in self.envs]

    @classmethod
    def deserialize(cls, data: Data) -> None:
        raise NotImplementedError()
