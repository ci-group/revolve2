from revolve2.modular_robot.body.base import ActiveHingeSensor
from revolve2.modular_robot.sensor_state import ModularRobotSensorState, ActiveHingeSensorState


class PhysicalSensorState(ModularRobotSensorState):
    """A Class for using physical sensors."""

    def get_active_hinge_sensor_state(
        self, sensor: ActiveHingeSensor
    ) -> ActiveHingeSensorState:
        pass
