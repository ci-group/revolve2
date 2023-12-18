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

    def __init__(self, debug: bool, dry: bool) -> None:
        """
        Initialize this object.

        :param debug: If debugging messages are activated.
        :param dry: If servo outputs are not propagated to the physical servos.
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
            for i in range(16, 32)
        ]

        self._debug = debug
        self._dry = dry

        if not self._dry:
            self._robohat = Robohat(
                self._SERVOASSEMBLY_1_CONFIG, self._SERVOASSEMBLY_2_CONFIG, 7
            )
            self._robohat.init(servoboard_1_datas_list, servoboard_2_datas_list)
            self._robohat.do_buzzer_beep()
            self._robohat.set_servo_direct_mode(_mode=True)

    def set_servo_targets(self, pins: list[int], targets: list[float]) -> None:
        """
        Set the target for multiple servos.

        This can be a fairly slow operation.

        :param pins: The GPIO pin numbers.
        :param targets: The target angles.
        """
        for pin, target in zip(pins, targets):
            if self._debug:
                print(f"{pin:03d} | {target}")

        if not self._dry:
            all_angles = [90.0] * 32
            angles = [90.0 + target / (2.0 * math.pi) * 360.0 for target in targets]
            for pin, angle in zip(pins, angles):
                all_angles[pin] = angle
            self._robohat.set_servo_multiple_angles(all_angles)

    def enable(self) -> None:
        """Start the robot."""
        if self._debug:
            print("Waking up servos.")
        if not self._dry:
            self._robohat.wakeup_servo()

    def disable(self) -> None:
        """
        Set the robot to low power mode.

        This disables all active modules and sensors.
        """
        if self._debug:
            print("Putting servos to sleep.")
        if not self._dry:
            self._robohat.put_servo_to_sleep()
