import asyncio
from brain.brain import Brain  # type: ignore # Will be added when brain is exported
import json
from pathlib import Path
import os
import json
import time
import pigpio
from typing import Any, List
import jsonschema
from dataclasses import dataclass


@dataclass
class Pin:
    pin: int
    invert: bool


class Controller:
    _brain: Brain
    _gpio: pigpio.pi
    _control_period: float
    _pins: List[Pin]

    def __init__(self):
        with open(
            os.path.join(Path(__file__).parent, "settings.json"), "r"
        ) as settings_file, open(
            os.path.join(Path(__file__).parent, "settings_schema.json"), "r"
        ) as settings_schema_file:
            settings = json.loads(settings_file.read())
            settings_schema = json.loads(settings_schema_file.read())
            jsonschema.validate(instance=settings, schema=settings_schema)

        self._brain = Brain()

        self._init_gpio(settings)

        self._control_period = 1.0 / settings["control_frequency"]

    def _init_gpio(self, settings: Any) -> None:
        self._gpio = pigpio.pi()

        gpio_settings = [gpio for gpio in settings["gpio"]]
        gpio_settings.sort(key=lambda gpio: gpio["dof"])
        i = -1
        for gpio in gpio_settings:
            if gpio["dof"] != i + 1:
                raise RuntimeError(
                    "GPIO pin settings are not a incremental list of degrees of freedom indices."
                )
            i += 1

        gpio_settings = [gpio["gpio_pin"] for gpio in gpio_settings]
        targets = self._brain.get_dof_targets()
        if len(gpio_settings) != len(targets):
            raise RuntimeError(
                "Number of degrees of freedom in brain does not match settings."
            )

        self._pins = [
            Pin(gpio_setting["gpio_pin"], gpio_settings["invert"])
            for gpio_setting in gpio_settings
        ]

        # TODO init gpios

        self._set_targets(targets)

    async def run(self) -> None:
        last_update_time = time.time()

        while True:
            await asyncio.sleep(self._control_period)
            current_time = time.time()
            elapsed_time = current_time - last_update_time
            last_update_time = current_time
            self._step(elapsed_time)

    def _step(self, dt: float) -> None:
        self._brain.step(dt)
        targets = self._brain.get_dof_targets()
        self._set_targets(targets)

    def _set_targets(self, target: List[float]) -> None:
        pass  # TODO


async def async_main() -> None:
    controller = Controller()
    await controller.run()


def main() -> None:
    import asyncio

    asyncio.run(async_main())


if __name__ == "__main__":
    main()
