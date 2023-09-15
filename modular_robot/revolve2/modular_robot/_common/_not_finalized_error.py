class NotFinalizedError(RuntimeError):
    """Error raised when querying a modular robot that has not yet been finalized."""

    def __init__(self) -> None:
        """Initialize this object."""
        super().__init__("Modular robot has not yet been finalized.")
