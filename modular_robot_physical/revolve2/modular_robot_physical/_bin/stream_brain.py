"""
Directly control a physical robot robot by providing commands over an input stream.

Commands:

setpins:
    {"cmd": "setpins", "pins": [{"pin": int, "target": float}]}
    Sets the target for the provided pins.
"""
import json
import sys
import traceback
from json.decoder import JSONDecodeError

import typed_argparse as tap

from ..physical_interfaces import HardwareType, PhysicalInterface, get_interface


class Args(tap.TypedArgs):
    """Arguments for the program."""

    hardware: HardwareType = tap.arg(help="The type of hardware this brain runs on.")
    debug: bool = tap.arg(help="Print debug information.")
    dry: bool = tap.arg(help="Skip GPIO output.")
    pins: list[int] = tap.arg(help="The GPIO pins that will be used.")
    m2m: bool = tap.arg(
        help="Enable machine-to-machine mode. Only print messages relevant to the protocol."
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
        m2m: bool,
    ) -> None:
        """
        Run the program.

        :param hardware_type: The type of hardware.
        :param debug: If debugging messages are activated.
        :param dry: If servo outputs are not propagated to the physical servos.:
        :param pins: The GPIO pins that will be used.
        :param m2m: Enable machine-to-machine mode. Only print messages relevant to the protocol.
        :raises RuntimeError: If shutdown was not clean.
        :raises CommandError: When a cli command is invalid.
        """
        try:
            print("1")

            if not m2m:
                print("Exit the program at any time by pressing Ctrl-C.")

            self._physical_interface = get_interface(
                hardware_type=hardware_type, debug=debug, dry=dry, pins=pins
            )

            print(json.dumps({"is_ok": True}))

            print("2")

            for line in sys.stdin:
                try:
                    parsed = json.loads(line)

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
                                        print(json.dumps({"is_ok": True}))
                                    case _:
                                        raise CommandError("Invalid command.")
                        case _:
                            raise CommandError("Invalid command.")
                except (CommandError, JSONDecodeError):
                    if not m2m:
                        print("Error parsing command:")
                        print(traceback.format_exc())
                        print("Recovered from error. You can send another command.")

        except KeyboardInterrupt:
            print("Program interrupted by user. Exiting..")
        finally:
            try:
                print("Setting hardware to low power mode..")
                self._physical_interface.to_low_power_mode()
                print("Done.")
            except:
                raise RuntimeError("Failed to set hardware to low power mode.")


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
        m2m=args.m2m,
    )


def main() -> None:
    """Run the script."""
    tap.Parser(Args).bind(runner).run()


if __name__ == "__main__":
    main()
