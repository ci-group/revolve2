import argparse
import asyncio
import sys
import pickle
import time
from dataclasses import dataclass

from ._physical_robot_config import PhysicalRobotConfig
from .physical_robot_control_interfaces import (
    Pca9685PhysicalControlInterface,
    PhysicalControlInterface,
    V1PhysicalControlInterface,
)
from .physical_robot_sensor_states import PhysicalSensorState


@dataclass
class _LogEntry:
    timestamp: int


class PhysicalRobotController:
    """Controller for the physical Robot."""

    _config: PhysicalRobotConfig
    _hardware_interface: PhysicalControlInterface
    _sensor_interface: PhysicalSensorState
    _stop: bool
    _log: list[_LogEntry]

    _debug: bool
    _log_file: str | None

    _control_period: float

    def __init__(self) -> None:
        """Initialize the Controller."""
        self._stop = False

    def main(self) -> None:
        """
        Execute the controller for the physical robot.

        :raises NotImplementedError: If for a certain robot type no physical control interface is defined.
        """
        try:
            parser = argparse.ArgumentParser()
            parser.add_argument(
                "physical_robot_config" # TODO: add type once set
            )  # os.fsencode is bytes  mybe think of other way
            parser.add_argument("--hardware", type=str)
            parser.add_argument(
                "--debug", help="Print debug information", action="store_true"
            )
            parser.add_argument(
                "--dry",
                help="If set, gpio output is skipped.",
                action="store_true",
            )
            parser.add_argument(
                "--log", help="If set, outputs controller log to this file.", type=str
            )
            parser.add_argument(
                "--all",
                help="Set all outputs provided in the config file to the given value.",
                type=str,
                choices=["min", "center", "max"],
            )
            args = parser.parse_args()
            self._log_file = args.log
            self._log = []

            self._control_period = 1 / self._config.control_frequency
            if isinstance(args.physical_robot_config, str):
                with open(args.physical_robot_config, "rb") as f:
                    self._config = PhysicalRobotConfig.from_pickle(pickle.dumps(f))
            elif isinstance(args.physical_robot_config, bytes):
                self._config = PhysicalRobotConfig.from_pickle(args.physical_robot_config)
            else:
                raise ValueError("PLease provide file path or bytes")



            self._sensor_interface = PhysicalSensorState()
            match args.hardware:
                case "v1":
                    self._hardware_interface = V1PhysicalControlInterface(
                        dry=args.dry,
                        debug=args.debug,
                        hinge_mapping=self._config.hinge_mapping,
                    )
                case "pca9685":
                    self._hardware_interface = Pca9685PhysicalControlInterface(
                        dry=args.dry,
                        debug=args.debug,
                        hinge_mapping=self._config.hinge_mapping,
                    )
                case _:
                    raise NotImplementedError(
                        "There is no physical interface for this type of hardware."
                    )

            match args.all:
                case "min":
                    self._hardware_interface.set_servo_targets(
                        [-1.0 for _ in self._config.hinge_mapping]
                    )
                    input("Press enter to stop.\n")
                case "max":
                    self._hardware_interface.set_servo_targets(
                        [1.0 for _ in self._config.hinge_mapping]
                    )
                    input("Press enter to stop.\n")
                case "center":
                    self._hardware_interface.set_servo_targets(
                        [0.0 for _ in self._config.hinge_mapping]
                    )
                    input("Press enter to stop.\n")
                case _:
                    self._hardware_interface.careful = True
                    user = input(
                        "Press enter to start controller. Press enter again to stop.\nOR\nType Q to stop now.\n"
                    )
                    if user == "q" or user == "Q":
                        self._hardware_interface.stop_pwm()
                    else:
                        asyncio.get_event_loop().run_until_complete(
                            asyncio.gather(
                                self._run_interface(), self._run_controller()
                            )
                        )
        except KeyboardInterrupt:
            self._hardware_interface.stop_pwm()

    async def _run_interface(self) -> None:
        await asyncio.get_event_loop().run_in_executor(None, sys.stdin.readline)
        self._stop = True

    async def _run_controller(self) -> None:
        last_update_time = time.time()
        controller = self._config.modular_robot.brain.make_instance()
        self._record_log(last_update_time)

        while not self._stop:
            await asyncio.sleep(self._control_period)
            current_time = time.time()
            elapsed_time = current_time - last_update_time
            last_update_time = current_time

            controller.control(
                elapsed_time,
                sensor_state=self._sensor_interface,
                control_interface=self._hardware_interface,
            )

            self._record_log(last_update_time)
        self._hardware_interface.stop_pwm()

    def _record_log(self, time: float) -> None:
        if self._log_file is not None:
            pass  # TODO: implement
