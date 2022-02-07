from .interface import Interface
from .controller import Controller
from .server import Server
from .cli import Cli
import logging
import asyncio
import argparse

PORT: int = 14875
PWM_FREQUENCY = 50


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
            # parser.add_argument("interface", type=str, choices=["server", "cli"])
            interface_subparsers = parser.add_subparsers(
                dest="interface", required=True
            )
            cli_parser = interface_subparsers.add_parser("cli")
            server_parser = interface_subparsers.add_parser("server")
            cli_parser.add_argument("config_file", type=str)

            args = parser.parse_args()

            self._controller = Controller(PWM_FREQUENCY)

            if args.interface == "cli":
                self._interface = Cli(self._controller, args.config_file)
            else:
                self._interface = Server(self._controller, PORT)

            interface_task = self._interface.run()
            controller_task = self._controller.run()

            finished, unfinished = asyncio.get_event_loop().run_until_complete(
                asyncio.wait(
                    [interface_task, controller_task],
                    return_when=asyncio.FIRST_COMPLETED,
                )
            )

            assert controller_task not in finished

            unfinished.add(self._controller.shutdown())

            asyncio.get_event_loop().run_until_complete(asyncio.wait(unfinished))

        except KeyboardInterrupt:
            exit(1)
