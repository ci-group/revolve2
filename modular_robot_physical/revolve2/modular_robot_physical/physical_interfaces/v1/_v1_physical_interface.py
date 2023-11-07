import math

import pigpio

from .._physical_interface import PhysicalInterface


class V1PhysicalInterface(PhysicalInterface):
    """Implementation of PhysicalInterface for V1 modular robots."""

    _PWM_FREQUENCY = 50
    _CENTER = 157.0
    _ANGLE60 = 64.0

    _debug: bool
    _dry: bool
    _pins: list[int]
    _gpio: pigpio.pi | None

    def __init__(self, debug: bool, dry: bool, pins: list[int]) -> None:
        """
        Initialize this object.

        :param debug: If debugging messages are activated.
        :param dry: If servo outputs are not propagated to the physical servos.:
        :param pins: The GPIO pins that will be used.
        :raises RuntimeError: If GPIOs could not initialize.
        """
        self._debug = debug
        self._dry = dry
        self._pins = pins

        if not self._dry:
            self._gpio = pigpio.pi()
            if not self._gpio.connected:
                raise RuntimeError("Failed to reach pigpio daemon.")

            for pin in self._pins:
                self._gpio.set_PWM_frequency(pin, self._PWM_FREQUENCY)
                self._gpio.set_PWM_range(pin, 2048)
                self._gpio.set_PWM_dutycycle(pin, 0)
        else:
            self._gpio = None

        if self._debug:
            print(f"Using PWM frequency {self._PWM_FREQUENCY}Hz")

    def set_servo_target(self, pin: int, target: float) -> None:
        """
        Set the target for a single Servo.

        :param pin: The GPIO pin number.
        :param target: The target angle.
        """
        if self._debug:
            print(f"{pin:03d} | {target}")

        if not self._dry:
            assert self._gpio is not None

            angle = self._CENTER + target / (1.0 / 3.0 * math.pi) * self._ANGLE60
            self._gpio.set_PWM_dutycycle(pin, angle)

    def to_low_power_mode(self) -> None:
        """
        Set the robot to low power mode.

        This disables all active modules and sensors.
        """
        if self._debug:
            print(
                "Turning off all pwm signals for pins that were used by this controller."
            )
        if not self._dry:
            assert self._gpio is not None

            for pin in self._pins:
                self._gpio.set_PWM_dutycycle(pin, 0)
