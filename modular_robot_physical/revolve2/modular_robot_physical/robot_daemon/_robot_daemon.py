import capnp

capnp.remove_import_hook()

import asyncio
import socket

from .._hardware_type import HardwareType
from .._standard_port import STANDARD_PORT
from ..physical_interfaces import PhysicalInterface, get_interface
from ._robo_server_impl import RoboServerImpl


class Server:
    """
    This handles a connection and passes everything to the actual RPC implementation.

    Copied from (slightly edited) https://github.com/capnproto/pycapnp/blob/5061cdd1eed5985eefb92c5843c0be3a8b061e0b/examples/async_server.py
    """

    _server: capnp.TwoPartyServer
    _stream_reader: asyncio.streams.StreamReader
    _stream_writer: asyncio.streams.StreamWriter

    _retry: bool

    async def _reader(self) -> bool:
        while self._retry:
            try:
                # Must be a wait_for so we don't block on read()
                data = await asyncio.wait_for(
                    self._stream_reader.read(4096), timeout=0.1
                )
            except asyncio.TimeoutError:
                continue
            except Exception as err:
                print("Unknown reader err: %s", err)
                return False
            await self._server.write(data)
        return True

    async def _writer(self) -> bool:
        while self._retry:
            try:
                # Must be a wait_for so we don't block on read()
                data = await asyncio.wait_for(self._server.read(4096), timeout=0.1)
                self._stream_writer.write(data.tobytes())
            except asyncio.TimeoutError:
                continue
            except Exception as err:
                print("Unknown reader err: %s", err)
                return False
        return True

    async def run_server(
        self,
        debug: bool,
        reader: asyncio.streams.StreamReader,
        writer: asyncio.streams.StreamWriter,
        physical_interface: PhysicalInterface,
    ) -> None:
        """
        Run the server.

        :param debug: Enable debug messages.
        :param reader: Stream reader.
        :param writer: Stream writer.
        :param physical_interface: Interface to the robot itself.
        """
        # Start TwoPartyServer using TwoWayPipe (only requires bootstrap)
        server_impl = RoboServerImpl(debug=debug, physical_interface=physical_interface)
        self._server = capnp.TwoPartyServer(bootstrap=server_impl)
        self._stream_reader = reader
        self._stream_writer = writer
        self._retry = True

        # Assemble reader and writer tasks, run in the background
        coroutines = [self._reader(), self._writer()]
        tasks = asyncio.gather(*coroutines, return_exceptions=True)

        while True:
            self._server.poll_once()
            # Check to see if reader has been sent an eof (disconnect)
            if self._stream_reader.at_eof():
                self._retry = False
                break
            await asyncio.sleep(0.01)

        # Make wait for reader/writer to finish (prevent possible resource leaks)
        await tasks

        # Clean up the server
        server_impl.cleanup()


class Program:
    """The program itself."""

    _debug: bool
    _dry: bool

    _has_client: bool
    _physical_interface: PhysicalInterface

    def __init__(self, debug: bool, dry: bool) -> None:
        """
        Initialize this object.

        :param debug: Enable debug messages.
        :param dry: Run in dry mode, not writing/reading hardware.
        """
        self._debug = debug
        self._dry = dry
        self._has_client = False

    async def _new_connection(
        self, reader: asyncio.streams.StreamReader, writer: asyncio.streams.StreamWriter
    ) -> None:
        """
        Handle a new connection.

        :param reader: Stream reader.
        :param writer: Stream writer.
        """
        if self._has_client:
            print(
                "Client connected, but still handling another client. Dropping new client.."
            )
            writer.close()
        else:
            self._has_client = True
            server = Server()
            await server.run_server(
                debug=self._debug,
                reader=reader,
                writer=writer,
                physical_interface=self._physical_interface,
            )
            self._has_client = False

    async def run(self, hardware_type: HardwareType) -> None:
        """
        Run the program.

        :param hardware_type: The type of hardware this runs on.
        """
        self._physical_interface = get_interface(
            hardware_type=hardware_type, debug=self._debug, dry=self._dry
        )

        server = await asyncio.start_server(
            self._new_connection, "*", STANDARD_PORT, family=socket.AF_INET
        )
        async with server:
            await server.serve_forever()


async def run_robot_daemon(debug: bool, dry: bool, hardware_type: HardwareType) -> None:
    """
    Run the server.

    :param debug: Enable debug messages.
    :param dry: Run in dry mode, not writing/reading hardware.
    :param hardware_type: The type of hardware this runs on.
    """
    await Program(debug=debug, dry=dry).run(hardware_type=hardware_type)
