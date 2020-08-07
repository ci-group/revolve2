
class SimulationWorker:

    def __init__(self, state: WorkerState = State.Available):
        self.state = State.Available