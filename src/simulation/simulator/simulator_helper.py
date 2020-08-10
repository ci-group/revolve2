from enum import Enum, auto

from nca.core.abstract.command import Command
from src.simulation.simulator.gazebo_connector_adapter import GazeboConnectorAdapter
from src.simulation.simulator.simulation_connector import SimulatorConnector

from revolve.evosphere.environment import Environment
from test.simulation_test.simulator.test_connector_adapter import TestConnectorAdapter


class SimulatorType(Enum):
    NONE = auto()
    GAZEBO = auto()
    V_REP = auto()


class TaskPriority(Enum):
    HIGH = auto()
    MEDIUM = auto()
    LOW = auto()


class RequestCommand(Command):

    def __init__(self, environment: Environment, simulator_type: SimulatorType,
                 task_priority: TaskPriority = TaskPriority.LOW):
        self.environment = environment
        self.simulator_type = simulator_type
        self.task_priority = task_priority

    def __eq__(self, other):
        return self.environment.path == other.environment.path and self.simulator_type == other.simulator_type

    def __hash__(self):
        return hash((self.environment.path, self.simulator_type))


class SimulatorFactory(object):

    def __init__(self, connection: RequestCommand):
        self.connection = connection

    def create(self) -> SimulatorConnector:
        if self.connection.simulator_type == SimulatorType.GAZEBO:
            return GazeboConnectorAdapter(self.connection.environment)
        elif self.connection.simulator_type == SimulatorType.NONE:
            return TestConnectorAdapter(self.connection.environment)
