from types import ModuleType
from xmlrpc.client import ProtocolError
from revolve2.object_controller import ObjectController
from typing import Optional, List, cast, Any
import asyncio
import importlib
import time
import pigpio
from dataclasses import dataclass
import jsonschema


@dataclass
class _Pin:
    pin: int
    invert: bool


class Controller:
    class UserError(RuntimeError):
        pass

    class SystemError(RuntimeError):
        pass

    _pwm_frequency: int

    _controller: Optional[ObjectController]
    _start_event: asyncio.Event
    _stopped_event: asyncio.Event
    _gpio: pigpio.pi
    _stop: bool
    _shutdown: bool

    _control_period: float
    _pins: List[_Pin]

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

    def __init__(self, pwm_frequency: int) -> None:
        self._pwm_frequency = pwm_frequency

        self._controller = None
        self._start_event = asyncio.Event()
        self._stopped_event = asyncio.Event()

        self._gpio = pigpio.pi()
        # TODO uncomment when running on real hardware
        # if not self._gpio.connected:
        #    raise self.SystemError("Failed to talk to GPIO daemon.")

        self._stop = True
        self._shutdown = False

    async def run(self) -> None:
        while True:
            await self._start_event.wait()

            if self._shutdown:
                return

            last_update_time = time.time()

            while True:
                if self._stop:
                    self._start_event.clear()
                    self._controller = None
                    self._stopped_event.set()
                    break
                await asyncio.sleep(self._control_period)
                current_time = time.time()
                elapsed_time = current_time - last_update_time
                last_update_time = current_time
                self._step(elapsed_time)

    async def load_controller(self, config: Any) -> None:
        if not self._stop:
            raise self.UserError("Cannot load controller. Controller is running.")

        self._controller = None

        try:
            jsonschema.validate(config, self._CONFIG_SCHEMA)

            controller_module = importlib.import_module(config["controller_module"])
            # TODO python is shit and cannot properly reload modules so if this is the same module as one loaded before its code is not updated.
            controller_type = getattr(controller_module, config["controller_type"])

            if not issubclass(controller_type, ObjectController):
                raise self.UserError("Controller is not an ObjectController")

            self._controller = controller_type.from_config(config["controller_config"])

            self._control_period = 1.0 / config["control_frequency"]
            self._init_gpio(config)
        except jsonschema.ValidationError as err:
            raise self.UserError("Invalid config.") from err
        except self.SystemError as err:
            raise err
        except Exception as err:
            raise self.UserError("Cannot load controller.") from err

    async def start_controller(self) -> None:
        """
        False if controller cannot be loaded.
        """

        if not self._stop:
            raise self.UserError("Cannot start controller. Already running.")

        if self._controller is None:
            raise self.UserError("Cannot start controller. No controller loaded.")

        self._stop = False
        self._stopped_event.clear()
        self._start_event.set()

    async def stop_controller(self) -> None:
        self._stop = True
        self._start_event.set()
        await self._stopped_event.wait()

    async def shutdown(self) -> None:
        await self.stop_controller()
        self._shutdown = True
        self._start_event.set()

    def _init_gpio(self, config: Any) -> None:
        assert self._controller is not None

        gpio_settings = [gpio for gpio in config["gpio"]]
        gpio_settings.sort(key=lambda gpio: cast(int, gpio["dof"]))
        i = -1
        for gpio in gpio_settings:
            if gpio["dof"] != i + 1:
                raise self.UserError(
                    "GPIO pin settings are not a incremental list of degrees of freedom indices."
                )
            i += 1

        targets = self._controller.get_dof_targets()
        if len(gpio_settings) != len(targets):
            raise self.UserError(
                "Number of degrees of freedom in brain does not match settings."
            )

        self._pins = [
            _Pin(gpio_setting["gpio_pin"], gpio_setting["invert"])
            for gpio_setting in gpio_settings
        ]

        try:
            for pin in self._pins:
                self._gpio.set_PWM_frequency(pin.pin, self._pwm_frequency)
                self._gpio.set_PWM_range(
                    pin.pin, 255
                )  # 255 is also the default, but just making sure
                self._gpio.set_PWM_dutycycle(pin.pin, 0)
        except AttributeError as err:
            raise self.SystemError("Could not initialize gpios.") from err

        self._set_targets(targets)

    def _step(self, dt: float) -> None:
        assert self._controller is not None

        self._controller.step(dt)
        targets = self._controller.get_dof_targets()
        self._set_targets(targets)

    def _set_targets(self, targets: List[float]) -> None:
        for pin, target in zip(self._pins, targets):
            self._gpio.set_PWM_dutycycle(pin.pin, target)
