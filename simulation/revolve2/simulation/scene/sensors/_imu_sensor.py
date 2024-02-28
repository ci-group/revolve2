from dataclasses import dataclass

from .._pose import Pose
from ._sensor import Sensor


@dataclass
class IMUSensor(Sensor):
    """
    An inertial measurement unit.

    Reports specific force(closely related to acceleration), angular rate(closely related to angularvelocity), and orientation.
    """

    pose: Pose
