import math

from robohatlib.hal.assemblyboard.PwmPlug import PwmPlug
from robohatlib.hal.assemblyboard.servo.ServoData import ServoData
from robohatlib.hal.assemblyboard.ServoAssemblyConfig import ServoAssemblyConfig
from robohatlib.Robohat import Robohat

from .._physical_interface import PhysicalInterface


class V2PhysicalInterface(PhysicalInterface):
    """Implementation of PhysicalInterface for V2 modular robots."""

    _SERVOASSEMBLY_1_CONFIG = ServoAssemblyConfig(
        _name="servoassembly_1",
        _sw1_pwm_address=0,
        _sw2_power_good_address=0,
        _cs_adc_angle_readout=PwmPlug.PWMPLUG_P3,
    )

    _SERVOASSEMBLY_2_CONFIG = ServoAssemblyConfig(
        _name="servoassembly_2",
        _sw1_pwm_address=1,
        _sw2_power_good_address=1,
        _cs_adc_angle_readout=PwmPlug.PWMPLUG_P4,
    )

    _debug: bool
    _dry: bool

    _robohat: Robohat

    def __init__(self, debug: bool, dry: bool, pins: list[int], careful: bool) -> None:
        """
        Initialize this object.

        :param debug: If debugging messages are activated.
        :param dry: If servo outputs are not propagated to the physical servos.
        :param pins: The GPIO pins that will be used.
        :param careful: Enable careful mode, which slowly steps the servo to its target, instead of going as fast as possible. This decreases current drawn by the motors, which might be necessary for some robots.
        """
        # Parameters for servo angle calculation.
        # These might be runtime parameters coming from some config in the future, so defining them in the init for now.
        INITIAL_VOLT_TO_ANGLE_FORMULA_A = 68.50117096018737
        INITIAL_VOLT_TO_ANGLE_FORMULA_B = -15.294412847106067

        servoboard_1_datas_list = [
            ServoData(
                _servo_nr=i,
                _min_time=500,
                _max_time=2500,
                _time_offset=0,
                _running_degree=180,
                _offset_degree=0,
                _formula_a=INITIAL_VOLT_TO_ANGLE_FORMULA_A,
                _formula_b=INITIAL_VOLT_TO_ANGLE_FORMULA_B,
            )
            for i in range(16)
        ]
        servoboard_2_datas_list = [
            ServoData(
                _servo_nr=i,
                _min_time=500,
                _max_time=2500,
                _time_offset=0,
                _running_degree=180,
                _offset_degree=0,
                _formula_a=INITIAL_VOLT_TO_ANGLE_FORMULA_A,
                _formula_b=INITIAL_VOLT_TO_ANGLE_FORMULA_B,
            )
            for i in range(16)
        ]

        self._debug = debug
        self._dry = dry

        if not self._dry:
            self._robohat = Robohat(
                self._SERVOASSEMBLY_1_CONFIG, self._SERVOASSEMBLY_2_CONFIG, 7
            )
            self._robohat.init(servoboard_1_datas_list, servoboard_2_datas_list)
            self._robohat.do_buzzer_beep()
        self._robohat.set_servo_direct_mode(careful)

    def set_servo_target(self, pin: int, target: float) -> None:
        """
        Set the target for a single Servo.

        :param pin: The GPIO pin number.
        :param target: The target angle.
        """
        if self._debug:
            print(f"{pin:03d} | {target}")

        if not self._dry:
            angle = target / (2 * math.pi) * 180 + 90
            self._robohat.set_servo_single_angle(pin, angle)

    def to_low_power_mode(self) -> None:
        """
        Set the robot to low power mode.

        This disables all active modules and sensors.
        """
        if self._debug:
            print(
                "Turning off all pwm signals for pins that were used by this controller."
            )
        self._robohat.put_servo_to_sleep()
