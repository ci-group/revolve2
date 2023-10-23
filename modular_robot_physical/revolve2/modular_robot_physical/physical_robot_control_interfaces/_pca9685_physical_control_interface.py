import time

from adafruit_servokit import ServoKit

from revolve2.modular_robot.body.base import ActiveHinge

from ._physical_control_interface import PhysicalControlInterface


class Pca9685PhysicalControlInterface(PhysicalControlInterface):
    """An Interface for the PCA9685 Physical Robot."""

    _PWM_FREQUENCY = 50
    _gpio: ServoKit

    _CENTER = 90.0
    _ANGLE60 = 60.0

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
        """
        super().__init__(dry=dry, debug=debug, hinge_mapping=hinge_mapping)

        if not self._dry:
            self._gpio = ServoKit(channels=16)
        if inverse_pin is None:
            self._pins = [self._Pin(pin_id, False) for pin_id in hinge_mapping.values()]
        else:
            self._pins = [
                self._Pin(pin_id, inverse_pin[pin_id])
                for pin_id in hinge_mapping.values()
            ]

        for pin in self._pins:
            self._gpio.servo[pin.pin].set_pulse_width_range(
                1
            )  # TODO: this is just a placeholder

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
            invert_mul = 1 if pin.invert else -1

            angle = self._CENTER + invert_mul * target * self._ANGLE60
            self._gpio.servo[pin.pin].angle = angle

        if self.careful:
            time.sleep(0.5)

    def stop_pwm(self) -> None:
        """Stop the signals and the robot."""
        if self._debug:
            print(
                "Turning off all pwm signals for pins that were used by this controller."
            )
        for pin in self._pins:
            if not self._dry:
                self._gpio.servo[pin.pin].fraction = None
