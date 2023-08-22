import logging


def setup_logging(level: int = logging.INFO) -> None:
    """
    Set up standard logging.

    :param level: The log level to use.
    """
    # Set up logging.
    # Each message has an associated 'level'.
    # By default, we are interested in messages of level 'info' and the more severe 'warning', 'error', and 'critical',
    # and we exclude the less severe 'debug'.
    # Furthermore, we specify the format in which we want the messages to be printed.
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )
