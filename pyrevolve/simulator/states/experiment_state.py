from pyrevolve.simulator.simulator import Simulator
from pyrevolve.simulator.states.simulation_state import SimulationState


class ExperimentState(SimulationState):


    def __init__(self):
        super().__init__(self)
        self.simulation = Simulator()