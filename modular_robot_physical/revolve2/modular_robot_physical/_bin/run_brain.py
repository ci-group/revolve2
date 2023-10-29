"""Run a brain on a physical robot."""
import pickle
from pathlib import Path

import typed_argparse as tap

from .._brain_runner import BrainRunner
from .._config import Config
from .._harware_type import HardwareType


class Args(tap.TypedArgs):
    """Arguments for the program."""

    config: Path = tap.arg(
        help="A file containing a pickled revolve.modular_robot_physical.Config object."
    )
    hardware: HardwareType = tap.arg(help="The type of hardware this brain runs on.")
    debug: bool = tap.arg(help="Print debug information.")
    dry: bool = tap.arg(help="Skip GPIO output.")
    log: Path | None = tap.arg(
        help="If set, controller output will be written to this file."
    )
    all: float | None = tap.arg(
        help="Instead of running the brain, set all servos to this angle (radians)."
    )


def runner(args: Args) -> None:
    """
    Run the program from the point were arguments were parsed.

    :param args: The parsed program arguments.
    :raises RuntimeError: If shutdown was not clean.
    """
    print("Exit the program at any time by pressing Ctrl-C.")
    try:
        with args.config.open("rb") as config_file:
            config = pickle.load(config_file)
            assert isinstance(config, Config)

        brain_runner = BrainRunner(
            hardware_type=args.hardware, config=config, debug=args.debug, dry=args.dry
        )

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
            print("Shutting down hardware..")
            brain_runner.shutdown()
            print("Done.")
        except:
            raise RuntimeError("Failed to shutdown hardware.")


def main() -> None:
    """Run the script."""
    tap.Parser(Args).bind(runner).run()


if __name__ == "__main__":
    main()
