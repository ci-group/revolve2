import argparse
import jsonschema
import json
from typing import Any, cast, List
import importlib
from revolve2.object_controller import ObjectController
import pigpio
from dataclasses import dataclass
import asyncio
import time
import sys


class Program:
    _CONFIG_SCHEMA = {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "type": "object",
        "properties": {
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
            "controller_config": {},
        },
        "required": [
            "controller_module",
            "controller_type",
            "control_frequency",
            "gpio",
            "controller_config",
        ],
    }

    _PWM_FREQUENCY = 50

    @dataclass
    class _Pin:
        pin: int
        invert: bool

    _controller: ObjectController
    _control_period: float
    _pins: List[_Pin]

    _gpio: pigpio.pi

    _stop: bool

    def __init__(self) -> None:
        self._stop = False

    def main(self) -> None:
        parser = argparse.ArgumentParser()
        parser.add_argument("config_file", type=str)
        args = parser.parse_args()

        with open(args.config_file) as file:
            config = json.load(file)
        jsonschema.validate(config, self._CONFIG_SCHEMA)

        self._load_controller(config)

        input("Press enter to start controller..")

        asyncio.get_event_loop().run_until_complete(
            asyncio.gather(self._run_interface(), self._run_controller())
        )

    async def _run_interface(self) -> None:
        print("Press enter to stop controller..")
        await asyncio.get_event_loop().run_in_executor(None, sys.stdin.readline)
        self._stop = True

    async def _run_controller(self) -> None:
        last_update_time = time.time()
        while not self._stop:
            await asyncio.sleep(self._control_period)
            current_time = time.time()
            elapsed_time = current_time - last_update_time
            last_update_time = current_time

            self._controller.step(elapsed_time)
            targets = self._controller.get_dof_targets()
            self._set_targets(targets)

    def _load_controller(self, config: Any) -> None:
        controller_module = importlib.import_module(config["controller_module"])
        controller_type = getattr(controller_module, config["controller_type"])

        if not issubclass(controller_type, ObjectController):
            raise ValueError("Controller is not an ObjectController")

        self._controller = controller_type.from_config(config["controller_config"])

        self._control_period = 1.0 / config["control_frequency"]
        self._init_gpio(config)

    def _init_gpio(self, config: Any) -> None:
        gpio = pigpio.pi()
        if not self._gpio.connected:
            raise RuntimeError("Failed to reach pigpio daemon.")

        gpio_settings = [gpio for gpio in config["gpio"]]
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

        try:
            for pin in self._pins:
                gpio.set_PWM_frequency(pin.pin, self._PWM_FREQUENCY)
                gpio.set_PWM_range(
                    pin.pin, 255
                )  # 255 is also the default, but just making sure
                gpio.set_PWM_dutycycle(pin.pin, 0)
        except AttributeError as err:
            raise RuntimeError("Could not initialize gpios.") from err

        self._set_targets(targets)

    def _set_targets(self, targets: List[float]) -> None:
        for pin, target in zip(self._pins, targets):
            self._gpio.set_PWM_dutycycle(pin.pin, (target + 1.0) / 2.0 * 255)


def main() -> None:
    Program().main()


if __name__ == "__main__":
    main()
