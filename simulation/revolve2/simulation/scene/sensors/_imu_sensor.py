from dataclasses import dataclass, field

from .._pose import Pose
from ._sensor import Sensor


@dataclass
class IMUSensor(Sensor):
    """
    An inertial measurement unit.

    Reports specific force(closely related to acceleration), angular rate(closely related to angularvelocity), and orientation.
    """

    pose: Pose
    type: str = field(
        default="imu"
    )  # The type attribute is used for the translation into XML formats.
