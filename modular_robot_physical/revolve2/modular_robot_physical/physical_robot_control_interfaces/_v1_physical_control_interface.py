import time

import pigpio

from revolve2.modular_robot.body.base import ActiveHinge

from ._physical_control_interface import PhysicalControlInterface


class V1PhysicalControlInterface(PhysicalControlInterface):
    """An Interface for the V1 Physical Robot."""

    _PWM_FREQUENCY = 50
    _gpio: pigpio.pi

    _CENTER = 157.0
    _ANGLE60 = 64.0

    def __init__(
        self,
        debug: bool,
        dry: bool,
        hinge_mapping: dict[ActiveHinge, int],
        inverse_pin: dict[int, bool] | None,
    ) -> None:
        """
        Initialize the PhysicalInterface.

        :param debug: If debugging messages are activated.
        :param dry: If dry.
        :param hinge_mapping: The modular robots hinges mapped to servos of the physical robot.
        :param inverse_pin: If pins are inversed.
        :raises RuntimeError: If GPIOs could not initialize.
        """
        super().__init__(dry=dry, debug=debug, hinge_mapping=hinge_mapping)

        if not self._dry:
            self._gpio = pigpio.pi()
            if not self._gpio.connected:
                raise RuntimeError("Failed to reach pigpio daemon.")

        if inverse_pin is None:
            self._pins = [self._Pin(pin_id, False) for pin_id in hinge_mapping.values()]
        else:
            self._pins = [
                self._Pin(pin_id, inverse_pin[pin_id])
                for pin_id in hinge_mapping.values()
            ]

        if self._debug:
            print(f"Using PWM frequency {self._PWM_FREQUENCY}Hz")

        if not self._dry:
            try:
                for pin in self._pins:
                    self._gpio.set_PWM_frequency(pin.pin, self._PWM_FREQUENCY)
                    self._gpio.set_PWM_range(pin.pin, 2048)
                    self._gpio.set_PWM_dutycycle(pin.pin, 0)
            except AttributeError as err:
                raise RuntimeError("Could not initialize gpios.") from err

    def stop_pwm(self) -> None:
        """Stop the signals and the robot."""
        if self._debug:
            print(
                "Turning off all pwm signals for pins that were used by this controller."
            )
        for pin in self._pins:
            if not self._dry:
                self._gpio.set_PWM_dutycycle(pin.pin, 0)

    def set_servo_targets(self, targets: list[float]) -> None:
        """
        Set the targets for servos.

        :param targets: The servos targets.
        """
        if self._debug:
            print("Setting pins to:")
            print("pin | target (clamped -1 <= t <= 1)")
            print("---------------")

        for i, target in enumerate(targets):
            self.set_servo_target(pin_id=i, target=target)

    def set_servo_target(self, pin_id: int, target: float) -> None:
        """
        Set the target for a single Servo.

        :param pin_id: The servos pin id.
        :param target: The target angle.
        """
        pin = self._pins[pin_id]
        if self._debug:
            print(f"{pin.pin:03d} | {target}")

        if not self._dry:
            invert_mul = 1.0 if pin.invert else -1.0

            angle = self._CENTER + (
                invert_mul * min(1.0, max(-1.0, target)) * self._ANGLE60
            )
            self._gpio.set_PWM_dutycycle(pin.pin, angle)

        time.sleep(0.5*self.careful)
