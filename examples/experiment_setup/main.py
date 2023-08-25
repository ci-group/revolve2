"""
Set up a trivial experiment with muliple repetitions.

You learn:
- Setting up an experiment.
- Basic use of logging.
- Experiment repetitions.
- About the random number generator and reproducible experiments.
- In particular, you will NOT learn how to save your experiment results.
  Either use your own preferred method, or look at the SQLAlchemy database abstraction commonly used in Revolve2.
"""

from revolve2.standard_resources.logging import setup_logging
import numpy as np
import config
import logging
from revolve2.standard_resources.rng import seed_from_string, make_rng


def run_experiment(num_samples: int, probability: int) -> None:
    # Create a list where we will store the success ratio for each repetition.
    success_ratios = []

    for repetition in range(config.NUM_REPETITIONS):
        logging.info(
            f"Running experiment ( repetition {repetition} num_samples {num_samples} probability {probability} )"
        )

        # First, we set up a random number generator, or 'rng'.
        # A rng is not actually random; it is an algorithm creating semi-randomness from a given starting number (the 'seed'), defined by you.
        # This is both very useful and a pitfall.
        # Using the same seed makes our experiment reproducible,
        # but using the same seed between two seperate experiments will generate the same numbers (usually bad).
        # As such we create a unique seed for each experiment and run.
        # Revolve2 helps you here by letting you define the seed as a string.

        # This string should be unique for your experiment and run
        rng_seed_str = f"trivial_experiment_repetition_{repetition}_num_samples_{num_samples}_probability_{probability}"
        rng_seed_int = seed_from_string(rng_seed_str)
        logging.info(f"Rng seed: {rng_seed_int} | {rng_seed_str}")

        # Create the rng using the seed.
        rng = make_rng(rng_seed_int)

        # Perform the experiment, sampling from a binomial distribution using the given paramaters.
        samples = rng.binomial(n=1, p=probability, size=num_samples)
        # Calculate the ratio of success and save it.
        success_ratios.append(np.sum(samples == 1) / num_samples)

    # Do some simple analysis using the performed repetitions.
    std = np.std(success_ratios)
    mean = np.mean(success_ratios)
    logging.info(f"mean {mean} std {std}")


def main() -> None:
    # Set up standard logging.
    # This decides the level of severity of logged messages we want to display.
    # By default this is 'INFO' or more severe, and 'DEBUG' is excluded.
    # Furthermore, a standard message layout is set up.
    # If logging is not set up, important messages can be missed.
    setup_logging()

    # Let's print a simple message.
    # We use the 'info' function to give the message the 'INFO' severity.
    logging.info("Starting program.")
    # The following message will be invisible.
    logging.debug(
        "This debug message is invisible, because we set our visible severity to 'INFO' and higher."
    )

    # Our experiment will be very trivial; sample a number of times with a certain probability and determine the ratio of success.
    # Several probabilities and number of samples will be observed, and each experiment will be repeated a few times.
    # These parameters are defined in 'config.py'.
    # Iterate over all parameters.
    for num_samples in config.NUM_SAMPLES:
        for probability in config.PROBABILITIES:
            # And run the experiment.
            run_experiment(
                num_samples=num_samples,
                probability=probability,
            )


# Below is a very important idiom in python that you should understand as a programmer.
# __name__ is only set to __main__ if it is directly ran as a script.
# If the file is imported from another file the main function is not called.
# If you do not perform this check the main function is ALWAYS called.
# This leads to all kinds of unintended behaviour that is hard to debug.
# See also various articles, such as https://stackoverflow.com/questions/419163/what-does-if-name-main-do
if __name__ == "__main__":
    main()
