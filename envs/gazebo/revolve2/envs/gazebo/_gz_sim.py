from pyrr import Vector3
from revolve2.core.physics_robot import PhysicsRobot


class _GzSim:
    async def reset() -> None:
        """
        Clear everything from the environment and reset everything to initial state.
        """
        raise NotImplementedError()

    async def set_world(self, world_file: str):  # TODO is this a file?
        raise NotImplementedError()

    async def insert_physics_robot(robot: PhysicsRobot, position: Vector3) -> None:
        raise NotImplementedError()
