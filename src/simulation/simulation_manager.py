from typing import Dict

from simulation.simulation_measures import SimulationMeasures
from test.evosphere.mock_ecosphere import MockEcosphere
from nca.core.abstract.configurations import SimulatorConfiguration
from revolve.evosphere.ecosphere import Ecosphere
from simulation.simulator.simulator_command import SimulationRequest
from simulation.simulator.simulator_factory import SimulatorFactory
from test.simulation.simulator.mock_measures import MockSimulationMeasures
from src.simulation.simulation_supervisor import SimulationSupervisor, ThreadedSimulationSupervisor


class SimulationManager:

    def __init__(self, threaded: bool = False):
        self.configuration = SimulatorConfiguration()

        if threaded:
            self.supervisor_type = ThreadedSimulationSupervisor
        else:
            self.supervisor_type = SimulationSupervisor

        self.supervisors: Dict[int, self.supervisor_type] = {}

    def simulate(self, request_command: SimulationRequest):
        actors = request_command.birth_clinic.create(request_command.agents, request_command.ecosphere)

        simulator = SimulatorFactory(request_command).create()

        if simulator is None:
            if isinstance(request_command.ecosphere, MockEcosphere):
                for actor in actors:
                    actor.measures = MockSimulationMeasures()

        if isinstance(request_command.ecosphere, Ecosphere):
            if request_command.id not in self.supervisors:
                self.supervisors[request_command.id] = self.supervisor_type(request_command)
            for actor in actors:
                actor.measures = self.supervisors[request_command.id].work(actor)

            results: Dict[int, SimulationMeasures] = self.supervisors[request_command.id].manager()
            for actor in actors:
                actor.fitness.add("ecosphere " + str(request_command.id), 0.0) # results[actor.id].fitness

    """
    def _find_available_supervisor(self):
        selected_supervisor = None

        for supervisor in self.supervisors:
            if selected_supervisor is None:
                selected_supervisor = supervisor
                continue

            if selected_supervisor.workload() > supervisor.workload():
                selected_supervisor = supervisor

        return selected_supervisor
    """
