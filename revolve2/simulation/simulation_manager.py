from typing import Dict

from revolve2.simulation.simulation_measures import SimulationMeasures
from test.evosphere.mock_ecosphere import MockEcosphere
from revolve2.abstract.configurations import SimulatorConfiguration
from revolve2.revolve.evosphere.ecosphere import Ecosphere
from revolve2.simulation.simulator.simulator_command import SimulationRequest
from revolve2.simulation.simulator.simulator_factory import SimulatorFactory
from test.simulation.simulator.mock_measures import MockSimulationMeasures
from src.simulation.simulation_supervisor import SimulationSupervisor, ThreadedSimulationSupervisor


class SimulationManager:

    def __init__(self, threads: int = 4):
        self.configuration = SimulatorConfiguration()
        self.threads = threads

        if self.threads > 1:
            self.supervisor_type = ThreadedSimulationSupervisor
            self.supervisors: Dict[int, ThreadedSimulationSupervisor] = {}
        else:
            self.supervisor_type = SimulationSupervisor
            self.supervisors: Dict[int, SimulationSupervisor] = {}

    def simulate(self, request_command: SimulationRequest):
        request_command.number_of_workers = self.threads

        results: Dict[int, SimulationMeasures] = {}

        simulator = SimulatorFactory(request_command).create()
        if simulator is None:
            if isinstance(request_command.ecosphere, MockEcosphere):
                for actor in request_command.agents:
                    results[actor.id] = MockSimulationMeasures()

        actors = request_command.birth_clinic.create(request_command.agents, request_command.ecosphere)
        if isinstance(request_command.ecosphere, Ecosphere):
            if request_command.id not in self.supervisors:
                self.supervisors[request_command.id] = self.supervisor_type(request_command)
            for actor in actors:
                self.supervisors[request_command.id].work(actor)

            results = self.supervisors[request_command.id].manager()
        return results

