import time

from ._config import Config
from ._harware_type import HardwareType
from .physical_interfaces import PhysicalInterface


class BrainRunner:
    """Interfaces with hardware to run brains or manually control the robot."""

    _config: Config
    _physical_interface: PhysicalInterface

    _initial_setup_delay: float | None
    """Delay between setting each active hinge to it's initial postion."""

    def __init__(
        self, hardware_type: HardwareType, config: Config, debug: bool, dry: bool
    ) -> None:
        """
        Initialize this object.

        :param hardware_type: The type of hardware.
        :param config: Configuration of the brain and knowledge of the physical robot.
        :param debug: Whether to print debug information.
        :param dry: If set, control inputs for the robot not be propogated to the hardware.
        :raises NotImplementedError: If the hardware type is not supported.
        """
        self._config = config

        match hardware_type:
            case HardwareType.v1:
                from .physical_interfaces.v1 import V1PhysicalInterface

                self._physical_interface = V1PhysicalInterface(
                    debug=debug,
                    dry=dry,
                    hinge_mapping=self._config.hinge_mapping,
                    inverse_pin=self._config.inverse_servos,
                )

                self._initial_setup_delay = 0.2
            case _:
                raise NotImplementedError()

    def set_all_active_hinges(self, target: float) -> None:
        """
        Set all servos to the specified target.

        :param target: The target in radians.
        """
        for active_hinge in self._config.hinge_mapping:
            self._physical_interface.control_interface.set_active_hinge_target(
                active_hinge=active_hinge.value, target=target
            )

    def set_active_hinges_initial_positions(self) -> None:
        """Set all servos to their initial positions."""
        for active_hinge in self._config.hinge_mapping:
            self._physical_interface.control_interface.set_active_hinge_target(
                active_hinge=active_hinge.value,
                target=self._config.initial_hinge_positions[active_hinge],
            )

            if self._initial_setup_delay is not None:
                time.sleep(self._initial_setup_delay)

    def run_brain(self) -> None:
        """Run the brain from the config."""
        control_period = 1 / self._config.control_frequency

        start_time = time.time()
        last_update_time = start_time
        controller = self._config.modular_robot.brain.make_instance()

        while (current_time := time.time()) - start_time < self._config.run_duration:
            time.sleep(control_period)

            elapsed_time = current_time - last_update_time
            last_update_time = current_time

            sensor_state = self._physical_interface.read_sensor_state()
            controller.control(
                elapsed_time,
                sensor_state=sensor_state,
                control_interface=self._physical_interface.control_interface,
            )

    def shutdown(self) -> None:
        """
        Gracefully shut down the hardware, moving to a low power state.

        This does not shut down the CPU, just the servos and such.
        """
        self._physical_interface.shutdown()
