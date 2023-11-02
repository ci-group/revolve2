import json
import time

import paramiko
import paramiko.channel

from .._config import Config
from ..physical_interfaces import HardwareType
from .._uuid_key import UUIDKey
from ._modular_robot_control_interface_impl import ModularRobotControlInterfaceImpl
from ._modular_robot_sensor_state_impl import ModularRobotSensorStateImpl
from revolve2.modular_robot.body.base import ActiveHinge
import socket


class Remote:
    """Remote control for physical modular robot that locally executes a brain and transmits it over ssh using `stream_brain`."""

    _INITIAL_SETUP_DELAY = 2.0

    _config: Config
    _stdin: paramiko.channel.ChannelStdinFile
    _stdout: paramiko.channel.ChannelFile
    _stderr: paramiko.channel.ChannelStderrFile

    _debug: bool

    def __init__(
        self,
        hostname: str,
        username: str,
        password: str,
        hardware_type: HardwareType,
        config: Config,
        debug: bool = False,
    ) -> None:
        """
        Initialize this object.

        :param hostname: Network hostname.
        :param username: SSH username.
        :param password: SSH password.
        :param hardware_type: The type of hardware of the physical modular robot.
        :param config: The modular robot configuration.
        :param debug: Whether to print debug information.
        """
        self._config = config
        self._debug = debug

        ssh_client = paramiko.SSHClient()
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh_client.connect(hostname=hostname, username=username, password=password)
        transport = ssh_client.get_transport()
        transport.set_keepalive(1)
        transport.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        #  --m2m
        actual_command = f"stream_brain --hardware {hardware_type.name} --pins {' '.join([str(pin) for pin in self._config.hinge_mapping.values()])}"
        self._stdin, self._stdout, self._stderr = ssh_client.exec_command(
            f'bash -l -c "{actual_command} 2>&1" | tee text.txt',
        )

    def check_for_errors(self):
        if self._stderr.channel.recv_stderr_ready():
            errors = self._stderr.read().decode()
            if errors:
                raise Exception(f"Error from remote command: {errors}")

    def set_active_hinges_initial_positions(self) -> None:
        """Set all servos to their initial positions."""
        for active_hinge in self._config.hinge_mapping:
            print(time.time())

            # Check if there were any errors in the remote program.
            self.check_for_errors()

            # Set active hinge to initial position.
            self.set_active_hinge_targets(
                [(active_hinge, self._config.initial_hinge_positions[active_hinge])]
            )

            # Wait a bit until setting the next active hinge.
            time.sleep(self._INITIAL_SETUP_DELAY)

    def set_active_hinge_targets(
        self, active_hinges_and_targets: list[tuple[UUIDKey[ActiveHinge], float]]
    ) -> None:
        """
        Set the target for active hinges.

        :param active_hinges_and_targets: The active hinges and their targets.
        :raises RuntimeError: In case of unexpected client response.
        """
        pins = [
            self._config.hinge_mapping[active_hinge]
            for active_hinge, _ in active_hinges_and_targets
        ]
        inverses = [self._config.inverse_servos.get(pin, False) for pin in pins]
        targets = [
            (-1.0 if inverse else 1.0)
            * min(max(target, -active_hinge.value.range), active_hinge.value.range)
            for (active_hinge, target), inverse in zip(
                active_hinges_and_targets, inverses
            )
        ]

        # Set targets on robot
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
        cmd_str = json.dumps(cmd) + "\n"

        if self._debug:
            print(f"Sending command: {cmd_str}")

        self._stdin.write(cmd_str)
        self._stdin.flush()

        # Read robot response
        # response = self._stdout.read()
        # print(response)
        # try:
        #     parsed = json.loads(response)
        # except JSONDecodeError:
        #     raise RuntimeError("Unexpected client response.")
        # match parsed:
        #     case {"is_ok": bool(is_ok)}:
        #         if not is_ok:
        #             raise RuntimeError("Error on robot setting servo targets.")
        #     case _:
        #         raise RuntimeError("Unexpected client response.")

    def run_brain(self) -> None:
        """Run the brain and remote."""
        try:
            self.set_active_hinges_initial_positions()

            control_period = 1 / self._config.control_frequency

            start_time = time.time()
            last_update_time = start_time
            controller = self._config.modular_robot.brain.make_instance()

            while (
                current_time := time.time()
            ) - start_time < self._config.run_duration:
                # Check if there were any errors in the remote program.
                self.check_for_errors()

                # Sleep until next control update
                time.sleep(control_period)

                elapsed_time = current_time - last_update_time
                last_update_time = current_time

                # Get targets from brain
                control_interface = ModularRobotControlInterfaceImpl()
                sensor_state = ModularRobotSensorStateImpl()
                controller.control(
                    elapsed_time,
                    sensor_state=sensor_state,
                    control_interface=control_interface,
                )

                self.set_active_hinge_targets(control_interface._set_active_hinges)
        except OSError:
            raise RuntimeError("Unexpected close of remote program.")
