"""Run a brain on a physical robot."""
import typed_argparse as tap

from .._harware_type import HardwareType
from ..physical_interfaces import PhysicalInterface


class Args(tap.TypedArgs):
    """Arguments for the program."""

    hardware: HardwareType = tap.arg(help="The type of hardware this brain runs on.")
    debug: bool = tap.arg(help="Print debug information.")
    dry: bool = tap.arg(help="Skip GPIO output.")
    pins: list[int] = tap.arg(help="The GPIO pins that will be used.")


class Program:
    """The program itself."""

    _physical_interface: PhysicalInterface

    def run(
        self, hardware_type: HardwareType, debug: bool, dry: bool, pins: list[int]
    ) -> None:
        """
        Run the program.

        :param hardware_type: The type of hardware.
        :param debug: If debugging messages are activated.
        :param dry: If servo outputs are not propagated to the physical servos.:
        :param pins: The GPIO pins that will be used.
        :raises NotImplementedError: When the provided hardware type is not supported.
        :raises RuntimeError: If shutdown was not clean.
        """
        print("Exit the program at any time by pressing Ctrl-C.")
        try:
            match hardware_type:
                case HardwareType.v1:
                    from ..physical_interfaces.v1 import V1PhysicalInterface

                    self._physical_interface = V1PhysicalInterface(
                        debug=debug,
                        dry=dry,
                        pins=pins,
                    )
                case _:
                    raise NotImplementedError()

            while True:
                pass

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
        hardware_type=args.hardware, debug=args.debug, dry=args.dry, pins=args.pins
    )


def main() -> None:
    """Run the script."""
    tap.Parser(Args).bind(runner).run()


if __name__ == "__main__":
    main()
