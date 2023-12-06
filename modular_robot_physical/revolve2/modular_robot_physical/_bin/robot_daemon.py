"""Physical modular robot daemon that provides a network interface to use the robot."""

import typed_argparse as tap

from .._hardware_type import HardwareType
from ..robot_daemon import run_robot_daemon


class Args(tap.TypedArgs):
    """Arguments for the program."""

    debug: bool = tap.arg(help="Print debug messages.", default=False)
    hardware: HardwareType = tap.arg(help="The type of hardware this brain runs on.")
    dry: bool = tap.arg(
        help="Run in dry mode, not doing anything with the robot hardware.",
        default=False,
    )


def main_with_args(args: Args) -> None:
    """
    Run the program from the point were arguments were parsed.

    :param args: The parsed program arguments.
    """
    run_robot_daemon(debug=args.debug, dry=args.dry, hardware_type=args.hardware)


def main() -> None:
    """Run the script."""
    tap.Parser(Args).bind(main_with_args).run()
