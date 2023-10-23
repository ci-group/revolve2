from robohatlib import Robohatgit
from robohatlib.hal.assemblyboard.servo.ServoData import ServoData
from robohatlib.hal.assemblyboard.ServoAssemblyConfig import ServoAssemblyConfig
from robohatlib.hal.assemblyboard.PwmPlug import PwmPlug

from revolve2.modular_robot.body.base import ActiveHinge

from ._physical_control_interface import PhysicalControlInterface


class V2PhysicalControlInterface(PhysicalControlInterface):
    """An Interface for the V2 Physical Robot."""

    _INITIAL_VOLT_TO_ANGLE_FORMULA_A = 68.50117096018737  # parameter A of the formula servo voltage to angle (y = Ax + B)
    _INITIAL_VOLT_TO_ANGLE_FORMULA_B = -15.294412847106067  # parameter B of the formula servo voltage to angle (y = Ax + B)
    _SERVOBOARD_1_DATAS_LIST = [
        ServoData(
            i,
            500,
            2500,
            0,
            180,
            0,
            _INITIAL_VOLT_TO_ANGLE_FORMULA_A,
            _INITIAL_VOLT_TO_ANGLE_FORMULA_B
        ) for i in range(16)
    ]
    _SERVOASSEMBLY_1_CONFIG = ServoAssemblyConfig(
        _name="servoassembly_1",
        _sw2_power_good_address=0,
        _cs_adc_angle_readout=PwmPlug.PWMPLUG_P3,
    )

    _SERVOBOARD_2_DATAS_LIST = [
        ServoData(
            i,
            500,
            2500,
            0,
            180,
            0,
            _INITIAL_VOLT_TO_ANGLE_FORMULA_A,
            _INITIAL_VOLT_TO_ANGLE_FORMULA_B
        ) for i in range(16)
    ]
    _SERVOASSEMBLY_2_CONFIG = ServoAssemblyConfig(
        _name="servoassembly_2",
        _sw1_pwm_address=1,
        _sw2_power_good_address=1,
        _cs_adc_angle_readout=PwmPlug.PWMPLUG_P4,
    )

    _gpio: Robohat

    _CENTER = 0.0
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
        :raises RuntimeError: If GPIOs could not initialize.
        """
        super().__init__(dry=dry, debug=debug, hinge_mapping=hinge_mapping)

        if not self._dry:
            self._gpio = Robohat(self._SERVOASSEMBLY_1_CONFIG, self._SERVOASSEMBLY_2_CONFIG, 7)
        self._gpio.set_servo_direct_mode(not self.careful, 0.0001)

        if inverse_pin is None:
            self._pins = [self._Pin(pin_id, False) for pin_id in hinge_mapping.values()]
        else:
            self._pins = [
                self._Pin(pin_id, inverse_pin[pin_id])
                for pin_id in hinge_mapping.values()
            ]

    def stop_pwm(self) -> None:
        """Stop the signals and the robot."""
        if self._debug:
            print(
                "Turning off all pwm signals for pins that were used by this controller."
            )
        self._gpio.do_system_shutdown()

    def set_servo_targets(self, targets: list[float]) -> None:
        """
        Set the targets for servos.

        :param targets: The servos targets.
        """
        if self._debug:
            print("Setting pins to:")
            print("pin | target (clamped -1 <= t <= 1)")
            print("---------------")
        self._gpio.set_servo_multiple_angles(targets)

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
            self._gpio.set_servo_single_angle(pin_id, angle)
