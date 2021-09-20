import unittest

from revolve2.simulation.simulator.simulator_command import SimulationRequest


class TestSimulatorConnector(unittest.TestCase):


    def test_simulation_requests(self):

        request1 = SimulationRequest(None, None, None)
        request2 = SimulationRequest(None, None, None)

        self.assertNotEqual(request1.id, request2.id)
