from pyrevolve.agent.agent import Agent
from pyrevolve.patterns.configuration import AgentConfiguration
from pyrevolve.patterns.sequential_identifier import AgentIdentifier


class BirthClinic():

    def __init__(self):
        self.robot_identifier = AgentIdentifier.Instance()
        self.configuration = AgentConfiguration()

    def create_agent(self) -> Agent:
        pass

    def build_agent(self) -> Agent:
        pass
