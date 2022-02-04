from .interface import Interface
from .controller import Controller
from .server import Server
from .cli import Cli
import logging
import asyncio
import argparse

PORT: int = 14875


class Program:
    _controller: Controller
    _interface: Interface

    def run(self) -> None:
        try:
            logging.basicConfig(
                level=logging.INFO,
                format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
            )

            parser = argparse.ArgumentParser()
            parser.add_argument("interface", type=str, choices=["server", "cli"])

            args = parser.parse_args()

            self._controller = Controller()

            if args.interface == "cli":
                self._interface = Cli(self._controller)
            else:
                self._interface = Server(self._controller, PORT)

            asyncio.get_event_loop().run_until_complete(
                asyncio.gather(self._interface.run(), self._controller.run())
            )
        except KeyboardInterrupt:
            exit(1)
