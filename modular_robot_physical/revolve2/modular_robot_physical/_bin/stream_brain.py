"""
Open a tcp server that will take commands to control the robot.

Commands:

setpins:
    {"cmd": "setpins", "pins": [{"pin": int, "target": float}]}
    Sets the target for the provided pins.
"""
import json
import socket

import typed_argparse as tap

from .._version import REVOLVE2_VERSION
from ..physical_interfaces import HardwareType, PhysicalInterface, get_interface


class Args(tap.TypedArgs):
    """Arguments for the program."""

    hardware: HardwareType = tap.arg(help="The type of hardware this brain runs on.")
    debug: bool = tap.arg(help="Print debug information.")
    dry: bool = tap.arg(help="Skip GPIO output.")
    pins: list[int] = tap.arg(help="The GPIO pins that will be used.")
    port: int = tap.arg(help="The port the open the stream on.")
    required_version: str | None = tap.arg(
        help="Assert whether the installed Revolve2 version matches the given value."
    )


class CommandError(Exception):
    """Error representing invalid cli commands."""

    pass


class Program:
    """The program itself."""

    _physical_interface: PhysicalInterface

    def run(
        self,
        hardware_type: HardwareType,
        debug: bool,
        dry: bool,
        pins: list[int],
        port: int,
    ) -> None:
        """
        Run the program.

        :param hardware_type: The type of hardware.
        :param debug: If debugging messages are activated.
        :param dry: If servo outputs are not propagated to the physical servos.:
        :param pins: The GPIO pins that will be used.
        :param port: The port to open the stream socket on.
        :raises RuntimeError: If shutdown was not clean.
        """
        try:
            print("Exit the program at any time by pressing Ctrl-C.")

            self._physical_interface = get_interface(
                hardware_type=hardware_type, debug=debug, dry=dry, pins=pins
            )

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as stream_socket:
                stream_socket.bind(("", port))
                stream_socket.listen()
                conn, addr = stream_socket.accept()
                with conn:
                    print("Connected by", addr)
                    while True:
                        data = conn.recv(1024)
                        if not data:
                            print("Stream closed by connection.")
                            break
                        command = data.decode("utf-8")
                        print(command)
                        self._handle_command(command)

        except KeyboardInterrupt:
            print("Program interrupted by user. Exiting..")
        finally:
            try:
                print("Setting hardware to low power mode..")
                self._physical_interface.to_low_power_mode()
                print("Done.")
            except:
                raise RuntimeError("Failed to set hardware to low power mode.")

    def _handle_command(self, command: str) -> None:
        parsed = json.loads(command)

        match parsed:
            case {"cmd": "setpins", "pins": list(pins_to_set)}:
                for pin in pins_to_set:
                    match pin:
                        case {
                            "pin": int(pin),
                            "target": float(target),
                        }:
                            self._physical_interface.set_servo_target(
                                pin=pin, target=target
                            )
                        case _:
                            raise CommandError("Invalid command.")
            case _:
                raise CommandError("Invalid command.")


def runner(args: Args) -> None:
    """
    Run the program from the point were arguments were parsed.

    :param args: The parsed program arguments.
    :raises RuntimeError: When required version does not match.
    """
    if args.required_version is not None and args.required_version != REVOLVE2_VERSION:
        raise RuntimeError(
            f"Program version is {REVOLVE2_VERSION} but does not match required version {args.required_version}."
        )

    Program().run(
        hardware_type=args.hardware,
        debug=args.debug,
        dry=args.dry,
        pins=args.pins,
        port=args.port,
    )


def main() -> None:
    """Run the script."""
    tap.Parser(Args).bind(runner).run()


if __name__ == "__main__":
    main()
