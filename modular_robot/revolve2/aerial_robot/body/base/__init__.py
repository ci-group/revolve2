"""Base Modules for Robots."""
from ._motor import Motor
from ._motor_sensor import MotorSensor
from ._core import Core

__all__ = [
    "Motor",
    "MotorSensor",
    "Core",
]
