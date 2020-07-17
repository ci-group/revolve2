from pyrevolve.evolutionary.fitness import Fitness


class Agent:

    def __init__(self, agent_id: int):
        self.id: int = agent_id


class EvolutionaryAgent(Agent):
    def __init__(self, agent_id: int, fitness: Fitness):
        super().__init__(agent_id)
        self.fitness: Fitness = fitness

