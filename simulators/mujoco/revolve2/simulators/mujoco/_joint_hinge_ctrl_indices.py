from dataclasses import dataclass


@dataclass
class JointHingeCtrlIndices:
    """Indices in the MuJoCo ctrl array for a hinge joint."""

    position: int
    velocity: int
