class IncompatibleError(RuntimeError):
    """
    Error raised when object cannot be loaded from a database because the database format is incompatible with the object.
    """

    def __init__(self) -> None:
        super().__init__("Database not compatible with this codebase.")
