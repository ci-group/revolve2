from ._physical_control_interface import PhysicalControlInterface
import pigpio
from typing import cast
import time
from revolve2.modular_robot.body.base import ActiveHinge


class V1PhysicalControlInterface(PhysicalControlInterface):
    """An Interface for the V1 Physical Robot."""
    _PWM_FREQUENCY = 50
    _gpio: pigpio.pi

    _CENTER = 157.0
    _ANGLE60 = 64.0

    def __init__(self, debug: bool, dry: bool, hinge_mapping: dict[ActiveHinge, int]) -> None:
        """
        Initialize the PhysicalInterface.

        :param debug: If debugging messages are activated.
        :param dry: If dry.
        :param hinge_mapping: The modular robots hinges mapped to servos of the physical robot.
        """
        super().__init__(dry=dry, debug=debug, hinge_mapping=hinge_mapping)

    def init_gpio(self, num_hinges: int) -> None:
        """
        Initialize the gpio.

        :param num_hinges: The amount of hinges for the modular robot.
        """
        if not self._dry:
            self._gpio = pigpio.pi()
            if not self._gpio.connected:
                raise RuntimeError("Failed to reach pigpio daemon.")

        gpio_settings = [gpio for gpio in self._config["gpio"]]
        gpio_settings.sort(key=lambda gpio: cast(int, gpio["dof"]))

        for i, gpio in enumerate(gpio_settings):
            if gpio["dof"] != i:
                raise ValueError(
                    "GPIO pin settings are not a incremental list of degrees of freedom indices."
                )

        if len(gpio_settings) != num_hinges:
            raise ValueError(
                "Number of degrees of freedom in brain does not match settings."
            )

        self._pins = [
            self._Pin(gpio_setting["gpio_pin"], gpio_setting["invert"])
            for gpio_setting in gpio_settings
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
                    invert_mul
                    * min(1.0, max(-1.0, target))
                    * self._ANGLE60
            )
            self._gpio.set_PWM_dutycycle(pin.pin, angle)

        if self.careful:
            time.sleep(0.5)
