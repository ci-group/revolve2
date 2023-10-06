from dataclasses import dataclass

from ._joint import Joint


@dataclass
class JointFixed(Joint):
    """
    A joint fixing the rigid bodies together rigidly.

    This makes them effectively a single rigid body.
    """
