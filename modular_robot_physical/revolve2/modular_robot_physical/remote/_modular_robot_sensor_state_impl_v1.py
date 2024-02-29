from revolve2.modular_robot.body.sensors import (
    ActiveHingeSensor,
    CameraSensor,
    IMUSensor,
)
from revolve2.modular_robot.sensor_state import (
    ActiveHingeSensorState,
    CameraSensorState,
    IMUSensorState,
    ModularRobotSensorState,
)


class ModularRobotSensorStateImplV1(ModularRobotSensorState):
    """Implementation of ModularRobotSensorState for v1 robots."""

    def get_active_hinge_sensor_state(
        self, sensor: ActiveHingeSensor
    ) -> ActiveHingeSensorState:
        """
        Get sensor states for Hinges.

        :param sensor: The sensor to query.
        :raises NotImplementedError: Always.
        """
        raise NotImplementedError("V1 hardware does not support sensors reading.")

    def get_imu_sensor_state(self, sensor: IMUSensor) -> IMUSensorState:
        """
        Get the state of the provided IMU sensor.

        :param sensor: The sensor.
        :raises NotImplementedError: Always.
        """
        raise NotImplementedError()

    def get_camera_sensor_state(self, sensor: CameraSensor) -> CameraSensorState:
        """
        Get the state of the camera sensor.

        :param sensor: The camera.
        :raises NotImplementedError: Always.
        """
        raise NotImplementedError()
