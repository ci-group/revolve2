from revolve2.nca.core.actor.agent import Agent
from revolve2.revolve.evosphere.biosphere import Ecosphere
from revolve2.simulation.simulator.adapter.simulation_adapters import AgentSimulatorAdapter
from src.simulation.simulation_measures import SimulationMeasures
from third_party.evoman.evoman_environment import EvomanEnvironment
from third_party.evoman.evoman_helper import EvomanConfiguration


class EvomanSimulatorAdapter(AgentSimulatorAdapter):

    def __init__(self, ecosphere: Ecosphere):
        super().__init__(ecosphere)
        self.environment = EvomanEnvironment(EvomanConfiguration(save_logs=False, headless=True))
        self.agent: Agent = None

    def _start_simulation(self) -> bool:
        # If not finished then it is started correctly
        return not self.environment.reset()[0]

    def _stop_simulation(self) -> bool:
        return self.environment.close()

    def _add_agent(self, agent: Agent):
        self.agent = agent

    def _remove_agent(self, agent: Agent):
        self.agent = None

    def _simulate(self) -> SimulationMeasures:
        done, fitness, observation = self.environment.reset()

        while not done:
            actions = self.agent.brain.activate(observation)
            done, fitness, observation = self.environment.step(actions)
            self.environment.render()

        measures = SimulationMeasures()
        measures.fitness = fitness

        return measures

    def _connect(self) -> bool:
        return True

    def _disconnect(self) -> bool:
        return False

    def _pause_simulation(self) -> bool:
        return False

    def get_simulation_time(self):
        return self.environment.time
