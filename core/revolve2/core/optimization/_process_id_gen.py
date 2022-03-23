class ProcessIdGen:
    _next_id: int

    def __init__(self) -> None:
        self._next_id = 0

    def gen(self) -> int:
        new_id = self._next_id
        self._next_id += 1
        return new_id

    def get_state(self) -> int:
        return self._next_id

    def set_state(self, state: int) -> None:
        self._next_id = state
