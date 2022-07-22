from dataclasses import dataclass

from pyrr import Quaternion, Vector3

from ._rigid_body import RigidBody


@dataclass
class Joint:
    """A rotary joint between two rigid bodies."""

    """Name of the joint."""
    name: str

    """One of the two rigid bodies."""
    body1: RigidBody

    """One of the two rigid bodies."""
    body2: RigidBody

    """Point the joint rotates about."""
    position: Vector3

    """Rotation of the joint."""
    orientation: Quaternion

    """Vector the joint rotates about."""
    axis: Vector3

    """
    Rotation range of the joint in radians.
    
    How much it can rotate to each side.
    """
    range: float

    """Maximum effort the joint can exert."""
    effort: float

    """Maximum velocity of the joint."""
    velocity: float
