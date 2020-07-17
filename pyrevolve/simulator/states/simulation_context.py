from pyrevolve.simulator.simulator import Simulator
from pyrevolve.simulator.states.simulation_state import SimulationState


class SimulationContext:

    def __init__(self):
        self.state: SimulationState = None

        self.simulator = Simulator()

