import time

from .._config import Config
from .._hardware_type import HardwareType
from ..physical_interfaces import PhysicalInterface, get_interface
from ._modular_robot_control_interface_impl import ModularRobotControlInterfaceImpl
from ._modular_robot_sensor_state_impl import ModularRobotSensorStateImpl


class BrainRunner:
    """Interfaces with hardware to run brains or manually control the robot."""

    _INITIAL_SETUP_DELAY = 0.2
    """
    Delay between setting each active hinge to it's initial postion.
    This is a workaround to prevent the v1 robot from directly crashing when all hinges suddenly move.
    """

    _config: Config
    _physical_interface: PhysicalInterface
    _control_interface: ModularRobotControlInterfaceImpl

    def __init__(
        self,
        hardware_type: HardwareType,
        config: Config,
        debug: bool,
        dry: bool,
        careful: bool,
    ) -> None:
        """
        Initialize this object.

        :param hardware_type: The type of hardware.
        :param config: Configuration of the brain and knowledge of the physical robot.
        :param debug: Whether to print debug information.
        :param dry: If set, control inputs for the robot not be propogated to the hardware.
        :param careful: Enable careful mode, which slowly steps the servo to its target, instead of going as fast as possible. This decreases current drawn by the motors, which might be necessary for some robots. This is only available for V2 robots.
        """
        self._config = config

        self._physical_interface = get_interface(
            hardware_type=hardware_type,
            debug=debug,
            dry=dry,
            pins=[pin for pin in config.hinge_mapping.values()],
            careful=careful,
        )

        self._control_interface = ModularRobotControlInterfaceImpl(
            hinge_mapping=config.hinge_mapping,
            inverse_pin=config.inverse_servos,
            physical_interface=self._physical_interface,
        )

    def set_all_active_hinges(self, target: float) -> None:
        """
        Set all servos to the specified target.

        :param target: The target in radians.
        """
        for active_hinge in self._config.hinge_mapping:
            self._control_interface.set_active_hinge_target(
                active_hinge=active_hinge.value, target=target
            )

    def set_active_hinges_initial_positions(self) -> None:
        """Set all servos to their initial positions."""
        for active_hinge in self._config.hinge_mapping:
            self._control_interface.set_active_hinge_target(
                active_hinge=active_hinge.value,
                target=self._config.initial_hinge_positions[active_hinge],
            )

            time.sleep(self._INITIAL_SETUP_DELAY)

    def run_brain(self) -> None:
        """Run the brain from the config."""
        control_period = 1 / self._config.control_frequency

        controller = self._config.modular_robot.brain.make_instance()

        start_time = time.time()
        last_update_time = start_time

        while (current_time := time.time()) - start_time < self._config.run_duration:
            next_update_at = last_update_time + control_period
            if current_time < next_update_at:
                time.sleep(next_update_at - current_time)
                last_update_time = next_update_at
                elapsed_time = control_period
            else:
                print(
                    f"WARNING: Loop is lagging {next_update_at - current_time} seconds behind the set update frequency. Is your control function too slow?"
                )
                elapsed_time = last_update_time - current_time
                last_update_time = current_time

            sensor_state = ModularRobotSensorStateImpl()
            controller.control(
                elapsed_time,
                sensor_state=sensor_state,
                control_interface=self._control_interface,
            )

    def shutdown(self) -> None:
        """
        Gracefully shut down the hardware, moving to a low power state.

        This does not shut down the CPU, just the servos and such.
        """
        self._physical_interface.to_low_power_mode()
