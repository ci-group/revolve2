"""Run a brain on a physical robot."""
import pickle
from pathlib import Path

import typed_argparse as tap

from .._config import Config
from ..brain_runner import BrainRunner
from ..physical_interfaces import HardwareType


class Args(tap.TypedArgs):
    """Arguments for the program."""

    config: Path = tap.arg(
        help="A file containing a pickled revolve.modular_robot_physical.Config object."
    )
    hardware: HardwareType = tap.arg(help="The type of hardware this brain runs on.")
    debug: bool = tap.arg(help="Print debug information.")
    dry: bool = tap.arg(help="Skip GPIO output.")
    all: float | None = tap.arg(
        help="Instead of running the brain, set all servos to this angle (radians)."
    )


def runner(args: Args) -> None:
    """
    Run the program from the point were arguments were parsed.

    :param args: The parsed program arguments.
    :raises RuntimeError: If shutdown was not clean.
    """
    with args.config.open("rb") as config_file:
        config = pickle.load(config_file)
        assert isinstance(config, Config)

    brain_runner = BrainRunner(
        hardware_type=args.hardware, config=config, debug=args.debug, dry=args.dry
    )

    try:
        print("Exit the program at any time by pressing Ctrl-C.")

        if args.all is not None:
            brain_runner.set_all_active_hinges(args.all)
            while True:
                pass
        else:
            print("Setting active hinges to initial positions..")
            brain_runner.set_active_hinges_initial_positions()
            print("Done. Press enter to start the brain.")
            input()
            brain_runner.run_brain()
            print("Running has finished.")

    except KeyboardInterrupt:
        print("Program interrupted by user. Exiting..")
    finally:
        try:
            print("Setting hardware to low power mode..")
            brain_runner.shutdown()
            print("Done.")
        except:
            raise RuntimeError("Failed to set hardware to low power mode.")


def main() -> None:
    """Run the script."""
    tap.Parser(Args).bind(runner).run()


if __name__ == "__main__":
    main()
