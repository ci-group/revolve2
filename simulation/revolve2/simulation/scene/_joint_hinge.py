from dataclasses import dataclass

from pyrr import Vector3

from ._joint import Joint


@dataclass
class JointHinge(Joint):
    """
    A hinge joint, also known as revolute joint.

    Rotates around a single axis.
    """

    axis: Vector3
    """Directional vector the joint rotates about."""

    range: float
    """
    Rotation range of the joint in radians.
    
    How much it can rotate to each side, in radians.
    So double this is the complete range of motion.
    """

    effort: float
    """Maximum effort the joint can exert, in newton-meter."""

    velocity: float
    """Maximum velocity of the joint, in radian per second."""

    armature: float
    """
    Armature of the joint.

    This represents the inertia of the motor itself when nothing is attached.
    """

    pid_gain_p: float
    """Proportional gain of the pid position controller."""

    pid_gain_d: float
    """Derivative gain of the pid position controller."""
