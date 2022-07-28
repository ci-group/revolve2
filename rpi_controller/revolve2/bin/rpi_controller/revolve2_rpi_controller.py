"""
Script to run an ActorController on a Raspberry Pi.

Installed as ``revolve2_rpi_controller``.
See ``revolve2_rpi_controller --help`` for usage.
"""

import argparse
import asyncio
import importlib
import json
import sys
import time
from dataclasses import dataclass
from typing import Any, List, Optional, Union, cast

import jsonschema
import pigpio
from adafruit_servokit import ServoKit
from revolve2.actor_controller import ActorController
from revolve2.serialization import StaticData


class Program:
    """Encapsulation of the program."""

    _CONFIG_SCHEMA = {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "type": "object",
        "properties": {
            "hardware": {"type": "string", "enum": ["hatv1", "pca9685"]},
            "controller_module": {"type": "string"},
            "controller_type": {"type": "string"},
            "control_frequency": {"type": "integer", "exclusiveMinimum": 0},
            "gpio": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "dof": {"type": "integer"},
                        "gpio_pin": {"type": "integer"},
                        "invert": {"type": "boolean"},
                    },
                    "required": ["dof", "gpio_pin", "invert"],
                },
            },
            "serialized_controller": {},
        },
        "required": [
            "hardware",
            "controller_module",
            "controller_type",
            "control_frequency",
            "gpio",
            "serialized_controller",
        ],
    }

    _PWM_FREQUENCY = 50

    @dataclass
    class _Pin:
        pin: int
        invert: bool

    _debug: bool
    _dry: bool  # if true, gpio output is skipped.
    _log_file: Optional[str]

    _controller: ActorController
    _control_period: float
    _pins: List[_Pin]

    _gpio: Union[pigpio.pi, ServoKit]

    _stop: bool

    _config: Any

    @dataclass
    class _LogEntry:
        timestamp: int
        serialized_controller: StaticData

    _log: List[_LogEntry]

    def __init__(self) -> None:
        """Initialize this object."""
        self._stop = False

    def main(self) -> None:
        """Run the program."""
        try:
            parser = argparse.ArgumentParser()
            parser.add_argument("config_file", type=str)
            parser.add_argument(
                "--debug", help="Print debug information.", action="store_true"
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

            self._debug = args.debug
            self._dry = args.dry
            self._log_file = args.log
            self._log = []

            with open(args.config_file) as file:
                self._config = json.load(file)
            jsonschema.validate(self._config, self._CONFIG_SCHEMA)

            self._load_controller()

            if args.all is not None:
                if args.all == "min":
                    target = -1.0
                elif args.all == "center":
                    target = 0.0
                else:
                    target = 1.0
                self._set_targets([target for _ in self._controller.get_dof_targets()])
                input("Press enter to stop.\n")
            else:
                self._set_targets(self._controller.get_dof_targets(), careful=True)
                user = input(
                    "Press enter to start controller. Press enter again to stop.\nOR\nType Q to stop now.\n"
                )
                if user == "q" or user == "Q":
                    self._stop_pwm()
                else:
                    asyncio.get_event_loop().run_until_complete(
                        asyncio.gather(self._run_interface(), self._run_controller())
                    )

            if self._log_file is not None:
                with open(self._log_file, "w") as log_file:
                    json.dump([entry.__dict__ for entry in self._log], log_file)
        except KeyboardInterrupt:
            # this is really ugly but whatever
            self._stop_pwm()

    async def _run_interface(self) -> None:
        await asyncio.get_event_loop().run_in_executor(None, sys.stdin.readline)
        self._stop = True

    async def _run_controller(self) -> None:
        last_update_time = time.time()

        self._record_log(last_update_time)

        while not self._stop:
            await asyncio.sleep(self._control_period)
            current_time = time.time()
            elapsed_time = current_time - last_update_time
            last_update_time = current_time

            self._controller.step(elapsed_time)
            targets = self._controller.get_dof_targets()
            self._set_targets(targets)

            self._record_log(last_update_time)

        self._stop_pwm()

    def _record_log(self, time: float) -> None:
        if self._log_file is not None:
            self._log.append(
                self._LogEntry(int(time * 1000), self._controller.serialize())
            )

    def _load_controller(self) -> None:
        controller_module = importlib.import_module(self._config["controller_module"])
        controller_type = getattr(controller_module, self._config["controller_type"])

        if not issubclass(controller_type, ActorController):
            raise ValueError("Controller is not an ActorController")

        self._controller = controller_type.deserialize(
            self._config["serialized_controller"]
        )

        self._control_period = 1.0 / self._config["control_frequency"]
        self._init_gpio()

    def _init_gpio(self) -> None:
        if not self._dry:
            if self._config["hardware"] == "hatv1":
                self._gpio = pigpio.pi()
                if not self._gpio.connected:
                    raise RuntimeError("Failed to reach pigpio daemon.")
            elif self._config["hardware"] == "pca9685":
                self._gpio = ServoKit(channels=16)
            else:
                raise NotImplementedError()

        gpio_settings = [gpio for gpio in self._config["gpio"]]
        gpio_settings.sort(key=lambda gpio: cast(int, gpio["dof"]))
        i = -1
        for gpio in gpio_settings:
            if gpio["dof"] != i + 1:
                raise ValueError(
                    "GPIO pin settings are not a incremental list of degrees of freedom indices."
                )
            i += 1

        targets = self._controller.get_dof_targets()
        if len(gpio_settings) != len(targets):
            raise ValueError(
                "Number of degrees of freedom in brain does not match settings."
            )

        self._pins = [
            self._Pin(gpio_setting["gpio_pin"], gpio_setting["invert"])
            for gpio_setting in gpio_settings
        ]

        if self._debug:
            print(f"Using PWM frequency {self._PWM_FREQUENCY}Hz")

        if not self._dry:
            if self._config["hardware"] == "hatv1":
                try:
                    for pin in self._pins:
                        self._gpio.set_PWM_frequency(pin.pin, self._PWM_FREQUENCY)
                        self._gpio.set_PWM_range(pin.pin, 2048)
                        self._gpio.set_PWM_dutycycle(pin.pin, 0)
                except AttributeError as err:
                    raise RuntimeError("Could not initialize gpios.") from err
            elif self._config["hardware"] == "pca9685":
                for pin in self._pins:
                    # self._gpio.servo[pin.pin].set_pulse_width_range(???)
                    # TODO can set this? to improve accuracy but I have no clue
                    pass
            else:
                raise NotImplementedError()

    def _set_targets(self, targets: List[float], careful: bool = False) -> None:
        if self._debug:
            print("Setting pins to:")
            print("pin | target (clamped -1 <= t <= 1)")
            print("---------------")
            for pin, target in zip(self._pins, targets):
                print(f"{pin.pin:03d} | {target}")

        for pin, target in zip(self._pins, targets):
            if not self._dry:
                if pin.invert:
                    invert_mul = -1
                else:
                    invert_mul = 1

                adjust_reversed_motor = (
                    -1
                )  # the motor is attached reversed by design so we need to inverse what it does.

                if self._config["hardware"] == "hatv1":
                    CENTER = 157
                    ANGLE60 = 64

                    angle = CENTER + (
                        adjust_reversed_motor
                        * invert_mul
                        * min(1, max(-1, target))
                        * ANGLE60
                    )

                    self._gpio.set_PWM_dutycycle(pin.pin, angle)
                elif self._config["hardware"] == "pca9685":
                    angle = 90 + adjust_reversed_motor * invert_mul * target * 60
                    self._gpio.servo[pin.pin].angle = angle
                else:
                    raise NotImplementedError()
            if careful:
                time.sleep(0.5)

    def _stop_pwm(self) -> None:
        if self._debug:
            print(
                "Turning off all pwm signals for pins that were used by this controller."
            )
        for pin in self._pins:
            if not self._dry:
                if self._config["hardware"] == "hatv1":
                    self._gpio.set_PWM_dutycycle(pin.pin, 0)
                elif self._config["hardware"] == "pca9685":
                    self._gpio.servo[pin.pin].fraction = None
                else:
                    raise NotImplementedError()


def main() -> None:
    """Run the script."""
    Program().main()


if __name__ == "__main__":
    main()
