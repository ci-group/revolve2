from __future__ import annotations

from dataclasses import dataclass
from typing import List

from pyrr import Quaternion, Vector3

from revolve2.serialization import Serializable, StaticData


@dataclass
class ActorState(Serializable):
    position: Vector3
    orientation: Quaternion

    def serialize(self) -> StaticData:
        return {
            "position": [
                float(self.position.x),
                float(self.position.y),
                float(self.position.z),
            ],
            "orientation": [
                float(self.orientation.x),
                float(self.orientation.y),
                float(self.orientation.z),
                float(self.orientation.w),
            ],
        }

    @classmethod
    def deserialize(cls, data: StaticData) -> ActorState:
        raise NotImplementedError()


@dataclass
class EnvironmentState(Serializable):
    actor_states: List[ActorState]

    def serialize(self) -> StaticData:
        return [state.serialize() for state in self.actor_states]

    @classmethod
    def deserialize(cls, data: StaticData) -> ActorState:
        raise NotImplementedError()


@dataclass
class State(Serializable):
    envs: List[EnvironmentState]

    def serialize(self) -> StaticData:
        return [env.serialize() for env in self.envs]

    @classmethod
    def deserialize(cls, data: StaticData) -> ActorState:
        raise NotImplementedError()
