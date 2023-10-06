"""Functions to work with logging in a standardized way."""

import logging


def setup_logging(level: int = logging.INFO, file_name: str | None = None) -> None:
    """
    Set up logging.

    :param level: The log level to use.
    :param file_name: If not None, also writes to this file.
    """
    # Set up logging.
    # Each message has an associated 'level'.
    # By default, we are interested in messages of level 'info' and the more severe 'warning', 'error', and 'critical',
    # and we exclude the less severe 'debug'.
    # Furthermore, we specify the format in which we want the messages to be printed.
    logging.basicConfig(
        level=level,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )
    if file_name is not None:
        logging.root.handlers.append(logging.FileHandler(file_name))
    logging.info("=======================================")
    logging.info("=======================================")
    logging.info("=======================================")
    logging.info("New log starts here.")
    logging.info("=======================================")
    logging.info("=======================================")
    logging.info("=======================================")
