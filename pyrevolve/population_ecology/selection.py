from pyrevolve.agent.agents import Agents


class Selection:

    def __init__(self, selection_size: int):
        self.selection_size: int = selection_size

    def select(self, agents: Agents) -> Agents:
        assert self.selection_size <= len(agents)
        return agents


class SteadyStateSelection(Selection):

    def __init__(self, selection_size: int):
        super().__init__(selection_size)

    def select(self, agents: Agents) -> Agents:
        super().select(agents)
        new_agents = Agents([])

        # TODO SS selection

        return new_agents


class TournamentSelection(Selection):

    def __init__(self, selection_size: int, tournament_size: int = 2):
        super().__init__(selection_size)
        self.tournament_size: int = tournament_size

    def select(self, agents: Agents) -> Agents:
        super().select(agents)
        new_agents = Agents([])

        #TODO tournament selection

        return new_agents


