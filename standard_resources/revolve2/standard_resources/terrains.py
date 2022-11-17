"""Standard terrains."""

from pyrr import Quaternion, Vector3
from revolve2.core.physics import Terrain
from revolve2.core.physics.running import geometry


def flat(size: Vector3 = Vector3([20.0, 20.0, 0.0])) -> Terrain:
    """
    Create a flat plane terrain.

    :param size: Size of the plane.
    :returns: The created terrain.
    """
    return Terrain(
        static_geometry=[
            geometry.Plane(
                position=Vector3(),
                orientation=Quaternion(),
                size=size,
            )
        ]
    )
