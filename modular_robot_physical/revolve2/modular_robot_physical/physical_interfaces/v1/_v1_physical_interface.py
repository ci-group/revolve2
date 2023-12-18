import math
import time

import pigpio

from .._physical_interface import PhysicalInterface


class V1PhysicalInterface(PhysicalInterface):
    """Implementation of PhysicalInterface for V1 modular robots."""

    _PWM_FREQUENCY = 50
    _CENTER = 157.0
    _ANGLE60 = 64.0
    _PINS = list(range(2, 28))

    _debug: bool
    _dry: bool

    _gpio: pigpio.pi | None

    def __init__(self, debug: bool, dry: bool) -> None:
        """
        Initialize this object.

        :param debug: If debugging messages are activated.
        :param dry: If servo outputs are not propagated to the physical servos.
        :raises RuntimeError: If GPIOs could not initialize.
        """
        self._debug = debug
        self._dry = dry

        if not self._dry:
            self._gpio = pigpio.pi()
            if not self._gpio.connected:
                raise RuntimeError("Failed to reach pigpio daemon.")

            for pin in self._PINS:
                self._gpio.set_PWM_frequency(pin, self._PWM_FREQUENCY)
                self._gpio.set_PWM_range(pin, 2048)
                self._gpio.set_PWM_dutycycle(pin, 0)
        else:
            self._gpio = None

        if self._debug:
            print(f"Using PWM frequency {self._PWM_FREQUENCY}Hz")

    def set_servo_targets(self, pins: list[int], targets: list[float]) -> None:
        """
        Set the target for multiple servos.

        This can be a fairly slow operation.

        :param pins: The GPIO pin numbers.
        :param targets: The target angles.
        """
        if not self._dry:
            assert self._gpio is not None
            for pin, target in zip(pins, targets):
                angle = self._CENTER + target / (1.0 / 3.0 * math.pi) * self._ANGLE60
                self._gpio.set_PWM_dutycycle(pin, angle)

    def enable(self) -> None:
        """Start the robot."""
        if not self._dry:
            assert self._gpio is not None
            for pin in self._PINS:
                self._gpio.set_PWM_dutycycle(pin, self._CENTER)
                print(f"setting {pin}..")
                time.sleep(0.1)

    def disable(self) -> None:
        """
        Set the robot to low power mode.

        This disables all active modules and sensors.
        """
        if self._debug:
            print("Turning off all pwm signals.")
        if not self._dry:
            assert self._gpio is not None

            for pin in self._PINS:
                self._gpio.set_PWM_dutycycle(pin, 0)
