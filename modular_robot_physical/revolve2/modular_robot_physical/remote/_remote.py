import asyncio
import time
from typing import Callable

import capnp

from revolve2.modular_robot.body.base import ActiveHinge

from .._config import Config
from .._protocol_version import PROTOCOL_VERSION
from .._standard_port import STANDARD_PORT
from .._uuid_key import UUIDKey
from ..robot_daemon_api import robot_daemon_protocol_capnp
from ._modular_robot_control_interface_impl import ModularRobotControlInterfaceImpl
from ._modular_robot_sensor_state_impl import ModularRobotSensorStateImpl


def _active_hinge_targets_to_pin_controls(
    config: Config, active_hinges_and_targets: list[tuple[UUIDKey[ActiveHinge], float]]
) -> list[robot_daemon_protocol_capnp.PinControl]:
    pins = [
        config.hinge_mapping[active_hinge]
        for active_hinge, _ in active_hinges_and_targets
    ]
    inverses = [config.inverse_servos.get(pin, False) for pin in pins]
    targets = [
        (-1.0 if inverse else 1.0)
        * min(max(target, -active_hinge.value.range), active_hinge.value.range)
        for (active_hinge, target), inverse in zip(active_hinges_and_targets, inverses)
    ]
    return [
        robot_daemon_protocol_capnp.PinControl(pin=pin, target=target)
        for pin, target in zip(pins, targets)
    ]


async def _run_remote_impl(
    config: Config,
    hostname: str,
    on_prepared: Callable[[], None],
    port: int,
    debug: bool,
) -> None:
    # Make controller
    controller = config.modular_robot.brain.make_instance()

    # Connect to robot
    try:
        connection = await capnp.AsyncIoStream.create_connection(
            host=hostname, port=port
        )
        client = capnp.TwoPartyClient(connection)
        service = client.bootstrap().cast_as(robot_daemon_protocol_capnp.RoboServer)
    except ConnectionRefusedError:
        raise ConnectionRefusedError("Could not connect to robot.")

    # Setup the robot and check protocol version
    setup_response: robot_daemon_protocol_capnp.SetupResponse = (
        await service.setup(
            robot_daemon_protocol_capnp.Setupargs(version=PROTOCOL_VERSION)
        )
    ).response
    if not setup_response.versionOk:
        raise RuntimeError("Protocol version does not match for robot.")

    # Set hinges to initial positions.
    pin_controls = _active_hinge_targets_to_pin_controls(
        config,
        [
            (active_hinge, config.initial_hinge_positions[active_hinge])
            for active_hinge in config.hinge_mapping
        ],
    )
    await service.control(
        robot_daemon_protocol_capnp.ControlCommands(pins=pin_controls)
    )

    # Fire prepared callback
    on_prepared()

    # Run the controller
    control_period = 1 / config.control_frequency

    start_time = time.time()
    last_update_time = start_time

    while (current_time := time.time()) - start_time < config.run_duration:
        # Sleep until next control update
        next_update_at = last_update_time + control_period
        if current_time < next_update_at:
            time.sleep(next_update_at - current_time)
            last_update_time = next_update_at
            elapsed_time = control_period
        else:
            print(
                f"WARNING: Loop is lagging {next_update_at - current_time} seconds behind the set update frequency. Is your control function too slow?"
            )
            elapsed_time = last_update_time - current_time
            last_update_time = current_time

        # Get targets from brain
        control_interface = ModularRobotControlInterfaceImpl()
        sensor_state = ModularRobotSensorStateImpl()
        controller.control(
            elapsed_time,
            sensor_state=sensor_state,
            control_interface=control_interface,
        )

        # Reading sensors will come in a later update.
        pin_controls = _active_hinge_targets_to_pin_controls(
            config, control_interface._set_active_hinges
        )
        await service.control(
            robot_daemon_protocol_capnp.ControlCommands(pins=pin_controls)
        )


def run_remote(
    config: Config,
    hostname: str,
    on_prepared: Callable[[], None] = lambda: None,
    port: int = STANDARD_PORT,
    debug: bool = False,
) -> None:
    """
    Control a robot remotely, running the controller on your local machine.

    :param config: The robot configuration.
    :param hostname: Hostname or IP of the robot.
    :param on_prepared: Callback for when everything is prepared, ready to run the actual controller. You can use this for timing when the actual controller starts.
    :param port: Port the robot daemon uses.
    :param debug: Enable debug messages.
    """
    asyncio.run(
        capnp.run(
            _run_remote_impl(
                config=config,
                hostname=hostname,
                on_prepared=on_prepared,
                port=port,
                debug=debug,
            )
        )
    )
