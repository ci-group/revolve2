class ProcessIdGen:
    """
    Generator for process ids unique within this program.

    Must always be called in the same order during reruns of the program or when attempting recovery, or you will lose reproducability.
    """

    _next_id: int

    def __init__(self) -> None:
        """Initialize this object."""
        self._next_id = 0

    def gen(self) -> int:
        """
        Generate a new unique id.

        :return: A unique id.
        """
        new_id = self._next_id
        self._next_id += 1
        return new_id

    def get_state(self) -> int:
        """
        Get the state of this generator so it can be serialize to a database.

        :return: The state of this generator.
        """
        return self._next_id

    def set_state(self, state: int) -> None:
        """
        Set the state of this generator so it can be deserialized from a database.

        :param state: The state to deserialize from.
        """
        self._next_id = state
