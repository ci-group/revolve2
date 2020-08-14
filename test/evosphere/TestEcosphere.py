from revolve.evosphere.world import Ecosphere
from simulation.simulator.simulator_helper import SimulatorType


class TestEcosphere(Ecosphere):

    def __init__(self, name: str = "test"):
        super().__init__(name)
        self.simulator_type = SimulatorType.NONE


class TestEcosphereA(Ecosphere):

    def __init__(self):
        super().__init__("A")


class TestEcosphereB(Ecosphere):

    def __init__(self):
        super().__init__("B")


class TestEcosphereC(Ecosphere):

    def __init__(self):
        super().__init__("C")