"""
Directly control a physical robot robot by providing commands over an input stream.

Commands:

setpins:
    {"cmd": "setpins", "pins": [{"pin": int, "target": float}]}
    Sets the target for the provided pins.
"""
import json

import typed_argparse as tap

from ..physical_interfaces import HardwareType, PhysicalInterface, get_interface
import socket


class Args(tap.TypedArgs):
    """Arguments for the program."""

    hardware: HardwareType = tap.arg(help="The type of hardware this brain runs on.")
    debug: bool = tap.arg(help="Print debug information.")
    dry: bool = tap.arg(help="Skip GPIO output.")
    pins: list[int] = tap.arg(help="The GPIO pins that will be used.")
    port: int = tap.arg(help="The port the open the stream on.")


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
        :raises CommandError: When a cli command is invalid.
        """
        try:
            print("Exit the program at any time by pressing Ctrl-C.")

            self._physical_interface = get_interface(
                hardware_type=hardware_type, debug=debug, dry=dry, pins=pins
            )

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as stream_socket:
                stream_socket.bind(("", port))  # Replace with appropriate port
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
    """
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
