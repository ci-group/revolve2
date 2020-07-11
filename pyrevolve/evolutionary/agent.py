from pyrevolve.evolutionary.fitness import Fitness


class Agent:

    def __init__(self, agent_id: int, fitness: Fitness):
        super().__init__()
        self.id: int = agent_id
        self.fitness: Fitness = fitness

