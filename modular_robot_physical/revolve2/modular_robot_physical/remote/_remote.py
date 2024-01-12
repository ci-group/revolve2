import asyncio
import time
from typing import Callable

import capnp

from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.sensor_state import ModularRobotSensorState

from .._config import Config
from .._hardware_type import HardwareType
from .._protocol_version import PROTOCOL_VERSION
from .._standard_port import STANDARD_PORT
from .._uuid_key import UUIDKey
from ..robot_daemon_api import robot_daemon_protocol_capnp
from ._modular_robot_control_interface_impl import ModularRobotControlInterfaceImpl
from ._modular_robot_sensor_state_impl_v1 import ModularRobotSensorStateImplV1
from ._modular_robot_sensor_state_impl_v2 import ModularRobotSensorStateImplV2


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
    manual_mode: bool,
) -> None:
    active_hinge_sensor_to_pin = {
        UUIDKey(key.value.sensor): pin
        for key, pin in config.hinge_mapping.items()
        if key.value.sensor
        if not None
    }

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
            robot_daemon_protocol_capnp.SetupArgs(
                version=PROTOCOL_VERSION, activePins=[x for x in range(32)]
            )
        )
    ).response
    if not setup_response.versionOk:
        raise RuntimeError("Protocol version does not match for robot.")
    match setup_response.hardwareType:
        case "v1":
            hardware_type = HardwareType.v1
        case "v2":
            hardware_type = HardwareType.v2
        case _:
            raise NotImplementedError()

    # Set hinges to initial positions.
    pin_controls = _active_hinge_targets_to_pin_controls(
        config,
        [
            (active_hinge, config.initial_hinge_positions[active_hinge])
            for active_hinge in config.hinge_mapping
        ],
    )
    match hardware_type:
        case HardwareType.v1:
            await service.control(
                robot_daemon_protocol_capnp.ControlArgs(setPins=pin_controls)
            )
        case HardwareType.v2:
            sensor_readings: robot_daemon_protocol_capnp.SensorReadings = (
                await service.controlAndReadSensors(
                    robot_daemon_protocol_capnp.ControlAndReadSensorsArgs(
                        setPins=pin_controls, readPins=[]
                    )
                )
            ).response
            print(f"Battery level is at {sensor_readings.battery*100.0}%.")

    # Fire prepared callback
    on_prepared()

    if manual_mode:
        print(
            "Press Ctrl-C to exit. type a value between -1 and 1 to manually set the target for each active hinge."
        )
        while True:
            try:
                target = float(input())
            except ValueError:
                print("Invalid target.")
                continue
            if target < -1 or target > 1:
                print("Invalid target.")
                continue
            pin_controls = _active_hinge_targets_to_pin_controls(
                config,
                [(active_hinge, target) for active_hinge in config.hinge_mapping],
            )

            await service.control(
                robot_daemon_protocol_capnp.ControlArgs(setPins=pin_controls)
            )

    else:
        # Run the controller
        control_period = 1 / config.control_frequency

        start_time = time.time()
        last_update_time = start_time

        battery_print_timer = 0.0

        sensor_state: ModularRobotSensorState

        # Get initial sensor state
        match hardware_type:
            case HardwareType.v1:
                sensor_state = ModularRobotSensorStateImplV1()
            case HardwareType.v2:
                pins = [pin for pin in active_hinge_sensor_to_pin.values()]
                sensor_readings = (
                    await service.readSensors(
                        robot_daemon_protocol_capnp.ReadSensorsArgs(readPins=pins)
                    )
                ).response
                sensor_state = ModularRobotSensorStateImplV2(
                    hinge_sensor_mapping=active_hinge_sensor_to_pin,
                    positions={
                        pin: position
                        for pin, position in zip(pins, sensor_readings.pins)
                    },
                )
            case _:
                raise NotImplementedError("Hardware type not supported.")

        while (current_time := time.time()) - start_time < config.run_duration:
            # Sleep until next control update
            next_update_at = last_update_time + control_period
            if current_time < next_update_at:
                await asyncio.sleep(next_update_at - current_time)
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
            controller.control(
                elapsed_time,
                sensor_state=sensor_state,
                control_interface=control_interface,
            )

            # Increase the timer for the battery value message
            battery_print_timer += elapsed_time

            # Reading sensors will come in a later update.
            pin_controls = _active_hinge_targets_to_pin_controls(
                config, control_interface._set_active_hinges
            )
            match hardware_type:
                case HardwareType.v1:
                    await service.control(
                        robot_daemon_protocol_capnp.ControlArgs(setPins=pin_controls)
                    )
                    sensor_state = ModularRobotSensorStateImplV1()
                case HardwareType.v2:
                    pins = [pin for pin in active_hinge_sensor_to_pin.values()]
                    sensor_readings = (
                        await service.controlAndReadSensors(
                            robot_daemon_protocol_capnp.ControlAndReadSensorsArgs(
                                setPins=pin_controls, readPins=pins
                            )
                        )
                    ).response
                    sensor_state = ModularRobotSensorStateImplV2(
                        hinge_sensor_mapping=active_hinge_sensor_to_pin,
                        positions={
                            pin: position
                            for pin, position in zip(pins, sensor_readings.pins)
                        },
                    )

                    if battery_print_timer > 5.0:
                        print(f"Battery level is at {sensor_readings.battery*100.0}%.")
                        battery_print_timer = 0.0
                case _:
                    raise NotImplementedError("Hardware type not supported.")


def run_remote(
    config: Config,
    hostname: str,
    on_prepared: Callable[[], None] = lambda: None,
    port: int = STANDARD_PORT,
    debug: bool = False,
    manual_mode: bool = False,
) -> None:
    """
    Control a robot remotely, running the controller on your local machine.

    :param config: The robot configuration.
    :param hostname: Hostname or IP of the robot.
    :param on_prepared: Callback for when everything is prepared, ready to run the actual controller. You can use this for timing when the actual controller starts.
    :param port: Port the robot daemon uses.
    :param debug: Enable debug messages.
    :param manual_mode: Enable manual controls for the robot, ignoring the brain.
    """
    asyncio.run(
        capnp.run(
            _run_remote_impl(
                config=config,
                hostname=hostname,
                on_prepared=on_prepared,
                port=port,
                debug=debug,
                manual_mode=manual_mode,
            )
        )
    )
