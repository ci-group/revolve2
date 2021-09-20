from abc import abstractmethod, ABC

from revolve2.abstract.sequential_identifier import SimulatorConnectionIdentifier
from revolve2.nca.core.actor.agent import Agent
from revolve2.nca.core.actor.individual import Individual

from revolve2.revolve.evosphere.biosphere import Ecosphere
from revolve2.revolve.robot.robot import Robot

from revolve2.simulation.simulator.simulator import Simulator
from revolve2.simulation.simulator.simulator_state import SimulatorState
from revolve2.simulation.simulation_measures import SimulationMeasures


class SimulatorAdapter(Simulator):

    port_identifier = SimulatorConnectionIdentifier()

    def __init__(self, ecosphere: Ecosphere):
        super().__init__()

        self.ecosphere = ecosphere

        self.port = self.port_identifier.id()

        self.state = SimulatorState.READY

        self.robot: Robot = None

    @abstractmethod
    def _simulate(self) -> SimulationMeasures:
        pass

    @abstractmethod
    def execute(self, individual: Individual) -> SimulationMeasures:
        pass


class RobotSimulatorAdapter(SimulatorAdapter, ABC):

    def execute(self, robot: Robot) -> SimulationMeasures:
        if self.state != SimulatorState.READY:
            raise Exception("Simulator is not ready")

        self.state = SimulatorState.RUNNING
        self._add_robot(robot)

        # wait for results
        measures = self._simulate()

        self._remove_robot(robot)
        self.state = SimulatorState.READY

        return measures

    @abstractmethod
    def _add_robot(self, robot: Robot):
        pass

    @abstractmethod
    def _remove_robot(self, robot: Robot):
        pass


class AgentSimulatorAdapter(SimulatorAdapter, ABC):

    def execute(self, agent: Agent) -> SimulationMeasures:
        if self.state != SimulatorState.READY:
            raise Exception("Simulator is not ready")

        self.state = SimulatorState.RUNNING
        self._add_agent(agent)

        # wait for results
        measures = self._simulate()

        self._remove_agent(agent)
        self.state = SimulatorState.READY

        return measures

    @abstractmethod
    def _add_agent(self, agent: Agent):
        pass

    @abstractmethod
    def _remove_agent(self, agent: Agent):
        pass
