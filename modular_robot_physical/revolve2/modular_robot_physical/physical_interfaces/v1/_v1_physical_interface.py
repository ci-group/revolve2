import math
import threading
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
    _careful: bool
    _careful_stepsize: float

    _gpio: pigpio.pi | None
    _set_targets: dict[int, float]  # The targets currently actually set.
    _targets: dict[int, float]  # The desired targets of the user.

    _enabled: bool
    _update_loop_thread: threading.Thread | None

    def __init__(
        self, debug: bool, dry: bool, careful: bool, careful_stepsize: float
    ) -> None:
        """
        Initialize this object.

        :param debug: If debugging messages are activated.
        :param dry: If servo outputs are not propagated to the physical servos.
        :param careful: Enable careful mode, which slowly steps the servo to its target, instead of going as fast as possible. This decreases current drawn by the motors, which might be necessary for some robots. This is only available for V2 robots.
        :param careful_stepsize: How much to step each iteration.
        :raises RuntimeError: If GPIOs could not initialize.
        """
        self._debug = debug
        self._dry = dry
        self._careful = careful
        self._careful_stepsize = careful_stepsize

        self._enabled = False
        self._update_loop_thread = None

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
        self._set_targets = {pin: 0.0 for pin in self._PINS}
        self._targets = {pin: 0.0 for pin in self._PINS}

        if self._debug:
            print(f"Using PWM frequency {self._PWM_FREQUENCY}Hz")

    def set_servo_target(self, pin: int, target: float) -> None:
        """
        Set the target for a single Servo.

        :param pin: The GPIO pin number.
        :param target: The target angle.
        """
        self._targets[pin] = target

    def _update_loop(self) -> None:
        while self._enabled:
            for pin in self._PINS:
                if self._careful:
                    set_target = self._set_targets[pin]
                    target = self._targets[pin]

                    self._set_targets[pin] = set_target + max(
                        min(target - set_target, self._careful_stepsize),
                        -self._careful_stepsize,
                    )
                else:
                    self._set_targets[pin] = self._targets[pin]

                target = self._set_targets[pin]

                if self._debug:
                    print(f"{pin:03d} | {target}")

                if not self._dry:
                    assert self._gpio is not None

                    angle = (
                        self._CENTER + target / (1.0 / 3.0 * math.pi) * self._ANGLE60
                    )
                    self._gpio.set_PWM_dutycycle(pin, angle)

            time.sleep(1 / 60)

    def enable(self) -> None:
        """Start the robot."""
        if self._enabled:
            print("Already enabled.")
            return

        self._enabled = True
        self._update_loop_thread = threading.Thread(target=self._update_loop)
        self._update_loop_thread.start()

    def disable(self) -> None:
        """
        Set the robot to low power mode.

        This disables all active modules and sensors.
        """
        if not self._enabled:
            print("Already disabled.")
            return
        assert self._update_loop_thread is not None

        if self._debug:
            print("Stopping background thread.")
        self._enabled = False
        self._update_loop_thread.join()
        self._update_loop_thread = None

        if self._debug:
            print(
                "Turning off all pwm signals for pins that were used by this controller."
            )
        if not self._dry:
            assert self._gpio is not None

            for pin in self._PINS:
                self._gpio.set_PWM_dutycycle(pin, 0)
