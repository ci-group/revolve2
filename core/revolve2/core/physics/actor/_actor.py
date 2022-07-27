from dataclasses import dataclass
from typing import List, Tuple

from pyrr import Quaternion, Vector3

from ._bounding_box import BoundingBox
from ._joint import Joint
from ._rigid_body import RigidBody


@dataclass
class Actor:
    """
    An actor to be used in a physics environment.

    Consists of a set of rigid bodies and joints connecting them which can possibly move dynamically.
    """

    bodies: List[RigidBody]
    joints: List[Joint]

    def calc_aabb(self) -> BoundingBox:
        """
        Calculate the axis aligned bounding box for this actor.

        This not the exact bounding box for the actor,
        but the smallest box the actor fits in that is aligned
        with the global axes.

        Not very efficient but it works and is fast enough for our use case

        :returns: The calculated bounding box.
        """
        xmin = 0
        xmax = 0
        ymin = 0
        ymax = 0
        zmin = 0
        zmax = 0

        for body in self.bodies:
            for collision in body.collisions:
                box = _Box(
                    (
                        Vector3(
                            [
                                -collision.bounding_box.x,
                                -collision.bounding_box.y,
                                -collision.bounding_box.z,
                            ]
                        )
                        / 2.0,
                        Vector3(
                            [
                                collision.bounding_box.x,
                                -collision.bounding_box.y,
                                -collision.bounding_box.z,
                            ]
                        )
                        / 2.0,
                        Vector3(
                            [
                                -collision.bounding_box.x,
                                collision.bounding_box.y,
                                -collision.bounding_box.z,
                            ]
                        )
                        / 2.0,
                        Vector3(
                            [
                                collision.bounding_box.x,
                                collision.bounding_box.y,
                                -collision.bounding_box.z,
                            ]
                        )
                        / 2.0,
                        Vector3(
                            [
                                -collision.bounding_box.x,
                                -collision.bounding_box.y,
                                collision.bounding_box.z,
                            ]
                        )
                        / 2.0,
                        Vector3(
                            [
                                collision.bounding_box.x,
                                -collision.bounding_box.y,
                                collision.bounding_box.z,
                            ]
                        )
                        / 2.0,
                        Vector3(
                            [
                                -collision.bounding_box.x,
                                collision.bounding_box.y,
                                collision.bounding_box.z,
                            ]
                        )
                        / 2.0,
                        Vector3(
                            [
                                collision.bounding_box.x,
                                collision.bounding_box.y,
                                collision.bounding_box.z,
                            ]
                        )
                        / 2.0,
                    ),
                )
                box.rotate(collision.orientation)
                box.translate(collision.position)
                box.rotate(body.orientation)
                box.translate(body.position)

                aabb = box.aabb()
                xmax = max(xmax, aabb.offset.x + aabb.size.x / 2.0)
                ymax = max(ymax, aabb.offset.y + aabb.size.y / 2.0)
                zmax = max(zmax, aabb.offset.z + aabb.size.z / 2.0)
                xmin = min(xmin, aabb.offset.x - aabb.size.x / 2.0)
                ymin = min(ymin, aabb.offset.y - aabb.size.y / 2.0)
                zmin = min(zmin, aabb.offset.z - aabb.size.z / 2.0)

        return BoundingBox(
            Vector3([xmax - xmin, ymax - ymin, zmax - zmin]),
            Vector3([(xmax + xmin) / 2.0, (ymax + ymin) / 2.0, (zmax + zmin) / 2.0]),
        )


@dataclass
class _Box:
    # The 8 coordinates of the box. Order is irrelevant.
    coordinates: Tuple[
        Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3, Vector3
    ]

    def rotate(self, rotation: Quaternion) -> None:
        self.coordinates = tuple(rotation * coord for coord in self.coordinates)  # type: ignore # TODO this whole file needs a cleanup

    def translate(self, offset: Vector3) -> None:
        self.coordinates = tuple(coord + offset for coord in self.coordinates)  # type: ignore # TODO this whole file needs a cleanup

    def aabb(self) -> BoundingBox:
        xmax = max([coord.x for coord in self.coordinates])
        ymax = max([coord.y for coord in self.coordinates])
        zmax = max([coord.z for coord in self.coordinates])
        xmin = min([coord.x for coord in self.coordinates])
        ymin = min([coord.y for coord in self.coordinates])
        zmin = min([coord.z for coord in self.coordinates])
        return BoundingBox(
            Vector3([xmax - xmin, ymax - ymin, zmax - zmin]),
            Vector3([(xmax + xmin) / 2.0, (ymax + ymin) / 2.0, (zmax + zmin) / 2.0]),
        )
