from simulation.environment import Environment
from simulation.simulator.simulator_helper import SimulatorType


class TestEnvironment(Environment):

    def __init__(self, name: str = "test"):
        super().__init__(name)
        self.simulator_type = SimulatorType.NONE


class TestEnvironmentA(TestEnvironment):

    def __init__(self):
        super().__init__("A")


class TestEnvironmentB(TestEnvironment):

    def __init__(self):
        super().__init__("B")


class TestEnvironmentC(TestEnvironment):

    def __init__(self):
        super().__init__("C")