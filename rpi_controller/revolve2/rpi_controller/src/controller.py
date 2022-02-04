from revolve2.object_controller import ObjectController
from typing import Optional, List, cast, Any
import asyncio
import traceback
import json
import importlib
import time
import pigpio
from dataclasses import dataclass


@dataclass
class _Pin:
    pin: int
    invert: bool


class Controller:
    _controller: Optional[ObjectController]
    _start_event: asyncio.Event
    _gpio: pigpio.pi
    _control_period: float
    _pins: List[_Pin]

    def __init__(self) -> None:
        self._controller = None
        self._start_event = asyncio.Event()

        self._gpio = pigpio.pi()
        # if not self._gpio.connected:
        #    raise RuntimeError("Failed to talk to GPIO daemon.")

    async def run(self) -> None:
        await self._start_event.wait()

        while True:
            await asyncio.sleep(self._control_period)
            current_time = time.time()
            elapsed_time = current_time - last_update_time
            last_update_time = current_time
            self._step(elapsed_time)

    async def start_controller(self, config_file: str) -> bool:
        """
        False if controller cannot be loaded.
        """

        if self._controller is not None:
            await self.stop_controller()

        try:
            with open(config_file) as file:
                config_str = file.read()
                config = json.loads(config_str)

                controller_module = importlib.import_module(config["controller_module"])
                controller_type = getattr(controller_module, config["controller_type"])

                if not issubclass(controller_type, ObjectController):
                    print("Controller is not an ObjectController")
                    return False

                self._controller = controller_type.from_config(
                    config["controller_config"]
                )

                self._control_period = 1.0 / config["control_frequency"]
                self._init_gpio(config)
        except Exception as err:
            print("Cannot load controller:")
            traceback.print_exc()
            return False

        self._start_event.set()
        return True

    async def stop_controller(self) -> bool:
        raise NotImplementedError("Stopping controller not yet implemented.")

    def _init_gpio(self, config: Any) -> None:
        gpio_settings = [gpio for gpio in config["gpio"]]
        gpio_settings.sort(key=lambda gpio: cast(int, gpio["dof"]))
        i = -1
        for gpio in gpio_settings:
            if gpio["dof"] != i + 1:
                raise RuntimeError(
                    "GPIO pin settings are not a incremental list of degrees of freedom indices."
                )
            i += 1

        targets = self._controller.get_dof_targets()
        if len(gpio_settings) != len(targets):
            raise RuntimeError(
                "Number of degrees of freedom in brain does not match settings."
            )

        self._pins = [
            _Pin(gpio_setting["gpio_pin"], gpio_setting["invert"])
            for gpio_setting in gpio_settings
        ]

        for pin in self._pins:
            self._gpio.set_PWM_frequency(pin.pin, config["pwm_frequency"])
            self._gpio.set_PWM_range(
                pin.pin, 255
            )  # 255 is also the default, but just making sure
            self._gpio.set_PWM_dutycycle(pin.pin, 0)

        self._set_targets(targets)

    def _step(self, dt: float) -> None:
        self._controller.step(dt)
        targets = self._controller.get_dof_targets()
        self._set_targets(targets)

    def _set_targets(self, targets: List[float]) -> None:
        for pin, target in zip(self._pins, targets):
            self._gpio.set_PWM_dutycycle(pin.pin, target)
