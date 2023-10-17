from ._physical_control_interface import PhysicalControlInterface
from adafruit_servokit import ServoKit
import time
from typing import cast
from revolve2.modular_robot.body.base import ActiveHinge


class Pca9685PhysicalControlInterface(PhysicalControlInterface):
    """An Interface for the PCA9685 Physical Robot."""
    _PWM_FREQUENCY = 50
    _gpio: ServoKit

    _CENTER = 90.0
    _ANGLE60 = 60.0

    def __init__(self, debug: bool, dry: bool, hinge_mapping: dict[ActiveHinge, int]) -> None:
        """
        Initialize the PhysicalInterface.

        :param debug: If debugging messages are activated.
        :param dry: If dry.
        :param hinge_mapping: The modular robots hinges mapped to servos of the physical robot.
        """
        super().__init__(dry=dry, debug=debug, hinge_mapping=hinge_mapping)

    def init_gpio(self) -> None:
        if not self._dry:
            self._gpio = ServoKit(channels=16)

        gpio_settings = [gpio for gpio in self._config["gpio"]]
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

    def set_servo_targets(self, targets: list[float], careful: bool = False) -> None:
        if self._debug:
            print("Setting pins to:")
            print("pin | target (clamped -1 <= t <= 1)")
            print("---------------")

        for i, target in enumerate(targets):
            self.set_servo_target(pin_id=i, target=target, careful=careful)

    def set_servo_target(self, pin_id: int, target: float, careful: bool) -> None:
        """
        Set the target for a single Servo.

        :param pin_id: The servos pin id.
        :param target: The target angle.
        :param careful: If careful.
        """
        pin = self._pins[pin_id]
        if self._debug:
            print(f"{pin.pin:03d} | {target}")

        if not self._dry:
            invert_mul = 1 if pin.inverted else -1

            angle = self._CENTER + invert_mul * target * self._ANGLE60
            self._gpio.servo[pin.pin].angle = angle

        if careful:
            time.sleep(0.5)

    def stop_pwm(self) -> None:
        if self._debug:
            print(
                "Turning off all pwm signals for pins that were used by this controller."
            )
        for pin in self._pins:
            if not self._dry:
                self._gpio.servo[pin.pin].fraction = None
