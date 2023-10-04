from dataclasses import dataclass

from ._pose import Pose
from ._rigid_body import RigidBody


@dataclass
class Camera:
    """Camera sensor."""

    parent: RigidBody
    """
    Parent rigid body.
    
    The camera will be rigidly attached to its parent, relatively using the camera's pose.    
    """

    pose: Pose
    """Pose of the geometry, relative to its parent rigid body."""
