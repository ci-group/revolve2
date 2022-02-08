import asyncio
from asyncio import IncompleteReadError, StreamWriter, StreamReader
import logging
import socket as socket_lib
from typing import Any
from .interface import Interface
from .controller import Controller
import json
import traceback
import jsonschema
from jsonschema import ValidationError


class _ProtocolError(RuntimeError):
    pass


class _DisconnectError(RuntimeError):
    pass


class Server(Interface):
    _port: int
    _socket: socket_lib.socket
    _client_is_connected: bool

    _MESSAGE_SCHEMA = {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "type": "object",
        "properties": {"type": {"type": "string"}},
        "required": ["type"],
    }

    _LOAD_CONTROLLER_SCHEMA = {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "type": "object",
        "properties": {"controller_config": {}},
        "required": ["controller_config"],
    }

    def __init__(self, controller: Controller, port: int) -> None:
        super().__init__(controller)
        self._port = port
        self._client_is_connected = False

    def init_socket(self, port: int) -> None:
        self._socket = socket_lib.socket(socket_lib.AF_INET, socket_lib.SOCK_STREAM)
        self._socket.bind(("127.0.0.1", port))
        self._socket.setblocking(False)

    async def handle_client(self, reader: StreamReader, writer: StreamWriter) -> None:
        logging.info("Client connected.")
        if self._client_is_connected:
            logging.info(
                "Another client was already connected. Dropping this new client."
            )
            writer.close()
            return

        self._client_is_connected = True

        try:
            while True:
                msg = await self._read_message(reader)
                jsonschema.validate(msg, self._MESSAGE_SCHEMA)

                if msg["type"] == "load_controller":
                    await self._handle_load_controller(msg, writer)
                else:
                    raise _ProtocolError(
                        "Client sent message not matching state of server."
                    )
        except (_ProtocolError, ValidationError) as err:
            logging.info(
                f"Error during communication with client: {traceback.format_exc()}"
            )
        except _DisconnectError:
            logging.info("Client disconnected.")
            pass
        except RuntimeError as err:
            logging.info(f"Unexpected error: {traceback.format_exc()}")

        writer.close()
        self._client_is_connected = False

    async def run(self) -> None:
        logging.info("Starting server. Waiting for client..")
        server = await asyncio.start_server(self.handle_client, "127.0.0.1", self._port)
        async with server:
            await server.serve_forever()

    async def _read_message(self, reader: StreamReader) -> Any:
        try:
            message_str = (await reader.readuntil("\n".encode("utf-8"))).decode("utf-8")
        except IncompleteReadError:
            raise _DisconnectError()

        try:
            return json.loads(message_str)
        except json.JSONDecodeError as err:
            raise _ProtocolError("Cannot load json message.") from err

    async def _handle_load_controller(self, msg: Any, writer: StreamWriter) -> None:
        jsonschema.validate(msg, self._LOAD_CONTROLLER_SCHEMA)
        config = msg["controller_config"]
        try:
            await self._controller.load_controller(config)
        except Controller.UserError as err:
            raise _ProtocolError("Could not load controller.") from err
        except Controller.SystemError as err:
            raise RuntimeError("Error in controller system.") from err
        writer.write(json.dumps({"type": "load_controller_complete"}).encode("utf-8"))
