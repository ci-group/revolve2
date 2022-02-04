import asyncio
import logging
import socket as socket_lib
from .interface import Interface
from .controller import Controller


class Server(Interface):
    _port: int
    _socket: socket_lib.socket

    def __init__(self, controller: Controller, port: int) -> None:
        super().__init__(controller)
        self._port = port

    def init_socket(self, port: int) -> None:
        self._socket = socket_lib.socket(socket_lib.AF_INET, socket_lib.SOCK_STREAM)
        self._socket.bind(("127.0.0.1", port))
        self._socket.setblocking(False)

    async def handle_client(self, reader, writer):
        logging.info("Client connected")

    async def run(self) -> None:
        logging.info("Starting server")
        server = await asyncio.start_server(self.handle_client, "127.0.0.1", self._port)
        async with server:
            await server.serve_forever()

        # logging.info("Initializing socket..")
        # self.init_socket(port)
        # logging.info("Socket initialized.")

        # while True:
        #     self._socket.listen()
        #     logging.info("Listening for connection..")
        #     conn, addr = self._socket.accept()
        #     logging.info("Accepted connection.")
        #     with conn:
        #         logging.info(f"Connected by {addr}")
        #         logging.info("Receiving JSON message..")
        #         message = ""
        #         found_end = False
        #         while not found_end:
        #             message += conn.recv(1024).decode()
        #             if message[-1] == "\0":
        #                 found_end = True
        #         logging.info("Received.")
        #         logging.debug(f"Message: {message}")
