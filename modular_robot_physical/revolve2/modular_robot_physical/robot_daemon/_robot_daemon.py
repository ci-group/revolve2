import asyncio
from typing import Any

import capnp

from .._hardware_type import HardwareType
from .._standard_port import STANDARD_PORT
from ..physical_interfaces import PhysicalInterface, get_interface
from ._robo_server_impl import RoboServerImpl


class _Program:
    """The program itself."""

    _debug: bool
    _dry: bool

    _has_client: bool
    _hardware_type: HardwareType
    _physical_interface: PhysicalInterface

    def __init__(self, debug: bool, dry: bool, hardware_type: HardwareType) -> None:
        """
        Initialize this object.

        :param debug: Enable debug messages.
        :param dry: Run in dry mode, not writing/reading hardware.
        :param hardware_type: The type of hardware this runs on.
        """
        self._debug = debug
        self._dry = dry
        self._has_client = False
        self._hardware_type = hardware_type

    async def _new_connection(self, stream: Any) -> None:
        """
        Handle a new connection.

        :param stream: Connection stream.
        """
        if self._has_client:
            if self._debug:
                print(
                    "Client connected, but still handling another client. Dropping new client.."
                )
            stream.close()
        else:
            self._has_client = True

            impl = RoboServerImpl(
                debug=self._debug,
                hardware_type=self._hardware_type,
                physical_interface=self._physical_interface,
            )
            await capnp.TwoPartyServer(stream, bootstrap=impl).on_disconnect()
            impl.cleanup()

            self._has_client = False

    async def run(self) -> None:
        """Run the program."""
        self._physical_interface = get_interface(
            hardware_type=self._hardware_type, debug=self._debug, dry=self._dry
        )  # Here we define the interface that controls the physical modular robot.

        server = await capnp.AsyncIoStream.create_server(
            self._new_connection, "*", STANDARD_PORT
        )
        async with server:
            await server.serve_forever()


def run_robot_daemon(debug: bool, dry: bool, hardware_type: HardwareType) -> None:
    """
    Run the server.

    :param debug: Enable debug messages.
    :param dry: Run in dry mode, not writing/reading hardware.
    :param hardware_type: The type of hardware this runs on.
    """
    asyncio.run(
        capnp.run(_Program(debug=debug, dry=dry, hardware_type=hardware_type).run())
    )
