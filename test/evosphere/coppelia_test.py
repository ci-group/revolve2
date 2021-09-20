import time

from revolve2.revolve.evosphere.coppelia import CoppeliaEvosphere
from revolve2.revolve.evosphere.ecosphere import CoppeliaEcosphere
from revolve2.simulation.simulator.adapter.vrep.coppelia_simulator_adapter import CoppeliaSimulatorAdapter

if __name__ == "__main__":

    vrep = CoppeliaSimulatorAdapter(CoppeliaEcosphere("robobo"))
    print(vrep.environment.robobo.simulator.client_id)
    print("finished")
    vrep._disconnect()

    evosphere = CoppeliaEvosphere()
    evosphere.evolve()
