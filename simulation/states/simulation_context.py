from simulation.simulator import Simulator
from simulation.states.simulation_state import SimulationState


class SimulationContext:

    def __init__(self):
        self.state: SimulationState = None

        self.simulator = Simulator()

