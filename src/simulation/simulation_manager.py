from typing import Dict
import threading

from evosphere.mock_ecosphere import MockEcosphere
from nca.core.abstract.configurations import SimulatorConfiguration
from revolve.evosphere.ecosphere import Ecosphere
from revolve.evosphere.evoman import EvomanEcosphere
from simulation.simulator.simulator_command import SimulateCommand
from simulation_test.simulator.mock_measures import MockSimulationMeasures
from src.simulation.simulation_supervisor import SimulationSupervisor


class SimulationManager:

    def __init__(self):
        self.configuration = SimulatorConfiguration()
        self.supervisors: Dict[SimulateCommand, SimulationSupervisor] = {}

    def simulate(self, request_command: SimulateCommand):
        #if request_command not in self.supervisors.keys():
        #    if not isinstance(request_command.ecosphere, EvomanEcosphere):
        #        self.supervisors[request_command] = SimulationSupervisor(request_command)

        actors = request_command.develop()

        if isinstance(request_command.ecosphere, MockEcosphere):
            for actor in actors:
                return MockSimulationMeasures()

        elif isinstance(request_command.ecosphere, EvomanEcosphere):
            for actor in actors:
                x = threading.Thread(target=request_command.ecosphere.run, args=[actor])
                x.start()
                x.join()

        elif isinstance(request_command.ecosphere, Ecosphere):
            for actor in actors:
                actor.measures = self.supervisors[request_command].work(actor, request_command)
                actor.performance(request_command.ecosphere.fitness(actor))

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