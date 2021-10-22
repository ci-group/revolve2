from ._gz_sim import _GzSim


class _GzPool:
    """
    A pool of Gazebo simulators that can be borrowed for exclusive use.
    """

    def __init__(self, simulator_count: int):
        pass

    async def borrow_simulator(self) -> _GzSim:
        raise NotImplementedError()

    def return_simulator(simulator: _GzSim) -> None:
        raise NotImplementedError()
