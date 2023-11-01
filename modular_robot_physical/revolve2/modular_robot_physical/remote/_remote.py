import paramiko
from ..physical_interfaces import HardwareType
import time
from ._modular_robot_control_interface_impl import ModularRobotControlInterfaceImpl
from ._modular_robot_sensor_state_impl import ModularRobotSensorStateImpl
from .._config import Config
import paramiko.channel
import json


class Remote:
    _config: Config
    _stdin: paramiko.channel.ChannelStdinFile
    _stdout: paramiko.channel.ChannelFile
    _stderr: paramiko.channel.ChannelStderrFile

    def __init__(
        self,
        hostname: str,
        username: str,
        password: str,
        hardware_type: HardwareType,
        dry: bool,
        config: Config,
    ) -> None:
        self._config = config

        # ssh_client = paramiko.SSHClient()
        # ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        # ssh_client.connect(hostname=hostname, username=username, password=password)
        # self.stdin, self.stdout, self.stderr = ssh_client.exec_command(
        #     f"stream_brain --hardware {hardware_type.name} {'--dry ' if dry else ''}--pins {' '.join([str(pin) for pin in self._config.hinge_mapping.values()])}"
        # )

    def run_brain(self) -> None:
        control_period = 1 / self._config.control_frequency

        start_time = time.time()
        last_update_time = start_time
        controller = self._config.modular_robot.brain.make_instance()

        while (current_time := time.time()) - start_time < self._config.run_duration:
            time.sleep(control_period)

            elapsed_time = current_time - last_update_time
            last_update_time = current_time

            control_interface = ModularRobotControlInterfaceImpl()
            sensor_state = ModularRobotSensorStateImpl()
            controller.control(
                elapsed_time,
                sensor_state=sensor_state,
                control_interface=control_interface,
            )

            pins = [
                self._config.hinge_mapping[active_hinge]
                for active_hinge in control_interface._set_active_hinges.keys()
            ]
            inverses = [self._config.inverse_servos.get(pin, False) for pin in pins]
            targets = [
                (-1.0 if inverse else 1.0)
                * min(max(target, -active_hinge.value.range), active_hinge.value.range)
                for active_hinge, target, inverse in zip(
                    control_interface._set_active_hinges.keys(),
                    control_interface._set_active_hinges.values(),
                    inverses,
                )
            ]

            cmd = {
                "cmd": "setpins",
                "pins": [
                    {
                        "pin": pin,
                        "target": target,
                    }
                    for pin, target in zip(pins, targets)
                ],
            }
            cmd_str = json.dumps(cmd)
            print(cmd_str)
