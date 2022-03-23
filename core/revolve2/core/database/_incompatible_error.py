class IncompatibleError(RuntimeError):
    def __init__(self) -> None:
        super().__init__("Database not compatible with this codebase.")
