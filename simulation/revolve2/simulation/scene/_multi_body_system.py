from __future__ import annotations

import uuid
from dataclasses import dataclass, field

import pyrr.aabb
from pyrr import Vector3

from ._aabb import AABB
from ._joint import Joint
from ._pose import Pose
from ._rigid_body import RigidBody
from .geometry import GeometryBox


@dataclass(kw_only=True)
class MultiBodySystem:
    """
    A (possibly cyclic) graph of interconnected rigid bodies, joints, and other objects, such as cameras.

    The first rigid body added is considered the root of the system.
    That is, if the system is static, that rigid body will be static.
    """

    _uuid: uuid.UUID = field(init=False, default_factory=uuid.uuid1)

    @property
    def uuid(self) -> uuid.UUID:
        """
        Get the uuid.

        :returns: The uuid.
        """
        return self._uuid

    pose: Pose
    """Pose of the system."""

    is_static: bool
    """
    Whether the root rigid body is static.
    
    I.e. its root (the first rigid body) is attached to the world and will not move or rotate.
    """

    _rigid_bodies: list[RigidBody] = field(default_factory=list, init=False)
    """Rigid bodies in this system."""

    _half_adjacency_matrix: list[Joint | None] = field(default_factory=list, init=False)
    """
    Adjacency matrix, defining joints between rigid bodies.
    
    The the indices of the list match the following indices in the adjacency matrix:

    joints | 0 1 2 3 4
         --|----------
         0 | - 0 2 5 9
         1 | 0 - 1 4 8
         2 | 2 1 - 3 7
         3 | 5 4 3 - 6
         4 | 9 8 7 6 -          
    """

    def _half_matrix_index(
        self, rigid_body1_list_index: int, rigid_body2_list_index: int
    ) -> int:
        assert rigid_body1_list_index != rigid_body2_list_index
        smallest_index = min(rigid_body1_list_index, rigid_body2_list_index)
        assert smallest_index >= 0
        largest_index = max(rigid_body1_list_index, rigid_body2_list_index)
        assert largest_index < len(self._rigid_bodies)

        base_index = ((largest_index - 1) * largest_index // 2) + (largest_index - 1)
        return base_index - smallest_index

    def add_rigid_body(self, rigid_body: RigidBody) -> None:
        """
        Add a rigid body to the system.

        :param rigid_body: The rigid body to add.
        """
        assert (
            not rigid_body._parent_info is not None
        ), "A rigid body can only be added to a single multi-body system."

        # Set parent info
        rigid_body._parent_info = RigidBody._ParentInfo(len(self._rigid_bodies))

        # Extend adjacency matrix
        self._half_adjacency_matrix.extend([None] * len(self._rigid_bodies))

        # Add rigid body
        self._rigid_bodies.append(rigid_body)

    def add_joint(self, joint: Joint) -> None:
        """
        Add a joint between two rigid bodies.

        :param joint: The joint to add.
        """
        assert (
            not joint._parent_info is not None
        ), "A joint can only be added to a single multi-body system."
        assert (
            joint.rigid_body1._parent_info is not None
            and joint.rigid_body1._parent_info.list_index < len(self._rigid_bodies)
        ), "First rigid body is not part of this multi-body system."
        assert (
            joint.rigid_body2._parent_info is not None
            and joint.rigid_body2._parent_info.list_index < len(self._rigid_bodies)
        ), "Second rigid body is not part of this multi-body system."
        assert (
            joint.rigid_body1._parent_info.list_index
            != joint.rigid_body2._parent_info.list_index
        ), "Cannot create a joint between a rigid body and itself."

        # Get the index in the adjacency matrix
        half_matrix_index = self._half_matrix_index(
            joint.rigid_body1._parent_info.list_index,
            joint.rigid_body2._parent_info.list_index,
        )
        assert (
            self._half_adjacency_matrix[half_matrix_index] is None
        ), "A joint already exists between these two rigid bodies."

        # Set parent info
        joint._parent_info = Joint._ParentInfo(half_matrix_index)

        # Assign the joint in the adjacency matrix
        self._half_adjacency_matrix[half_matrix_index] = joint

    def has_root(self) -> bool:
        """
        Check whether a root has been added.

        The root rigid body is the first rigid body that has been added,
        so this is only false if there are zero rigid bodies in this multi-body system.

        :returns: Whether there is a root joint.
        """
        return len(self._rigid_bodies) != 0

    @property
    def root(self) -> RigidBody:
        """
        Get the root rigid body of this multi-body system.

        The root rigid body is the first rigid body that has been added.

        :returns: The root rigid body.
        """
        assert len(self._rigid_bodies) != 0, "Root has not been added yet."
        return self._rigid_bodies[0]

    def get_joints_for_rigid_body(self, rigid_body: RigidBody) -> list[Joint]:
        """
        Get all joints attached to the provided rigid body.

        :param rigid_body: A previously added rigid body.
        :returns: The attached joints.
        """
        assert (
            rigid_body._parent_info is not None
            and rigid_body._parent_info.list_index < len(self._rigid_bodies)
        ), "Rigid body is not part of this multi-body system."

        half_matrix_indices = [
            self._half_matrix_index(
                rigid_body._parent_info.list_index, other_body_list_index
            )
            for other_body_list_index in range(len(self._rigid_bodies))
            if other_body_list_index != rigid_body._parent_info.list_index
        ]
        maybe_joints = [
            self._half_adjacency_matrix[half_matrix_index]
            for half_matrix_index in half_matrix_indices
        ]
        return [joint for joint in maybe_joints if joint is not None]

    def calculate_aabb(self) -> tuple[Vector3, AABB]:
        """
        Calculate the axis-aligned bounding box of this multi-body system when it is in T-pose.

        That is, when all joints are at position 0.
        Only box geometries are currently supported.

        :returns: Position, AABB
        :raises ValueError: If one of the geometries is not a box.
        """
        # Create list of all edges of geometry boxes.
        # We don't support anything but GeometryBox at this point.
        points: list[Vector3] = []
        for rigid_body in self._rigid_bodies:
            for geometry in rigid_body.geometries:
                if not isinstance(geometry, GeometryBox):
                    raise ValueError(
                        "AABB calculation currently only supports GeometryBox."
                    )

                for x_sign in [1, -1]:
                    for y_sign in [1, -1]:
                        for z_sign in [1, -1]:
                            points.append(
                                rigid_body.initial_pose.position
                                + rigid_body.initial_pose.orientation
                                * (
                                    geometry.pose.position
                                    + geometry.pose.orientation
                                    * (
                                        Vector3(
                                            [
                                                x_sign * geometry.aabb.size.x,
                                                y_sign * geometry.aabb.size.y,
                                                z_sign * geometry.aabb.size.z,
                                            ]
                                        )
                                        / 2.0
                                    )
                                )
                            )

        # Calculate AABB from the points.
        # This is simply the min and max between the points for every dimension.
        aabb = pyrr.aabb.create_from_points(points)
        xmin = aabb[0][0]
        ymin = aabb[0][1]
        zmin = aabb[0][2]
        xmax = aabb[1][0]
        ymax = aabb[1][1]
        zmax = aabb[1][2]

        # Return center and size of the AABB
        return Vector3([xmax + xmin, ymax + ymin, zmax + zmin]) / 2.0, AABB(
            Vector3([xmax - xmin, ymax - ymin, zmax - zmin])
        )
