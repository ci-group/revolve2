import math
import time
from typing import Sequence
from subprocess import call

import numpy as np
import pigpio
from numpy.typing import NDArray
from tempfile import NamedTemporaryFile
from PIL import Image

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
                if self._debug:
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

    def get_battery_level(self) -> float:
        """
        Get the battery level.

        :raises NotImplementedError: If getting the battery level is not supported on this hardware.
        """
        raise NotImplementedError("Getting battery level not supported on v1 harware.")

    def get_multiple_servo_positions(self, pins: Sequence[int]) -> list[float]:
        """
        Get the current position of multiple servos.

        :param pins: The GPIO pin numbers.
        :raises NotImplementedError: If getting the servo position is not supported on this hardware.
        """
        raise NotImplementedError("Getting servo position not supported on v1 harware.")

    def get_image(self) -> NDArray[np.int_]:
        """
        Get the current image of the camera.

        :return: The image.
        :raises ValueError: If getting the camera is not connected.
        """
        with NamedTemporaryFile(suffix=".jpeg") as tmp_file:
            file_name = tmp_file.name
            call(f"rpicam-jpeg -n -t 10 -o {file_name} --width 400 --height 400", shell=True)
            pil_image = Image.open(file_name)
        image: NDArray[np.int_] = np.asarray(pil_image)
        return image
