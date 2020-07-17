from pyrevolve.simulator.states.simulation_state import SimulationState


class AnalyzeState(SimulationState):

    def __init__(self):
        super().__init__(self)

        self.analyzer = analyzer