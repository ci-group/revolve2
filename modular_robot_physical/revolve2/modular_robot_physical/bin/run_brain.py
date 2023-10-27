"""Run a brain on a physical robot."""
from revolve2.modular_robot_physical import HardwareType, BrainRunner, Config
import typed_argparse as tap
from pathlib import Path
import pickle


class Args(tap.TypedArgs):
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
