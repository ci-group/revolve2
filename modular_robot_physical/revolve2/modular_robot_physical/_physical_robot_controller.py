from ._physical_robot_config import PhysicalRobotConfig
import argparse
from dataclasses import dataclass
from .physical_robot_control_interfaces import PhysicalControlInterface, V1PhysicalControlInterface, Pca9685PhysicalControlInterface
import asyncio
import sys
import time


@dataclass
class _LogEntry:
    timestamp: int


class PhysicalRobotController:
    _config: PhysicalRobotConfig
    _hardware_interface: PhysicalControlInterface
    _stop: bool
    _log: list[_LogEntry]

    _debug: bool
    _log_file: str | None

    _control_period: float

    def __init__(self) -> None:
        """Initialize the Controller."""
        self._stop = False

    def main(self) -> None:
        try:
            parser = argparse.ArgumentParser()
            parser.add_argument("physical_robot_config", type=bytes)
            parser.add_argument("--hardware", type=str)
            parser.add_argument(
                "--debug",
                help="Print debug information",
                action="store_true"
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
            self._config = PhysicalRobotConfig.from_pickle(args.physical_robot_config)
            match args.hardware:
                case "v1":
                    self._hardware_interface = V1PhysicalControlInterface(
                        dry=args["dry"],
                        debug=args["debug"],
                        hinge_mapping=self._config.hinge_mapping
                    )
                case "pca9685":
                    self._hardware_interface = Pca9685PhysicalControlInterface(
                        dry=args["dry"],
                        debug=args["debug"],
                        hinge_mapping=self._config.hinge_mapping
                    )
                case _:
                    raise NotImplementedError("There is no physical interface for this type of hardware.")

            match args.all:
                case "min":
                    self._hardware_interface.set_servo_targets([-1.0 for _ in self._config.hinge_mapping])  # TODO: hinge mapping is wrong here (just placeholder)
                case "max":
                    self._hardware_interface.set_servo_targets([1.0 for _ in self._config.hinge_mapping])
                case "center":
                    self._hardware_interface.set_servo_targets([0.0 for _ in self._config.hinge_mapping])
                case _:
                    #self._hardware_interface.set_servo_targets(self._config.hinge_mapping, careful=True)
                    user = input(
                        "Press enter to start controller. Press enter again to stop.\nOR\nType Q to stop now.\n"
                    )
                    if user == "q" or user == "Q":
                        self._hardware_interface.stop_pwm()
                    else:
                        asyncio.get_event_loop().run_until_complete(
                            asyncio.gather(self._run_interface(), self._run_controller())
                        )
        except KeyboardInterrupt:
            self._hardware_interface.stop_pwm()

    async def _run_interface(self) -> None:
        await asyncio.get_event_loop().run_in_executor(None, sys.stdin.readline)
        self._stop = True

    async def _run_controller(self) -> None:
        last_update_time = time.time()
        controller = self._config.modular_robot.brain
        self._record_log(last_update_time)

        while not self._stop:
            await asyncio.sleep(self._control_period)
            current_time = time.time()
            elapsed_time = current_time - last_update_time
            last_update_time = current_time

            controller.control(
                elapsed_time,
                sensor_state=None,
                control_interface=self._hardware_interface,
            )


            self._record_log(last_update_time)
        self._hardware_interface.stop_pwm()

    def _record_log(self, time: float) -> None:
        if self._log_file is not None:
            pass # TODO: implement