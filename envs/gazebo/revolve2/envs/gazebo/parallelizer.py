from .environment import Environment


class Parallelizer:
    """
    Manages multiple Gazebo simulators so multiple environments can be ran in parallel.
    """

    def new_environment(self) -> Environment:
        return Environment(self)

    async def _run_environment(environment: Environment) -> None # TODO result
        pass
