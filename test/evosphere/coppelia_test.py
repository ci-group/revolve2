import time

from revolve.evosphere.coppelia import CoppeliaEvosphere
from revolve.evosphere.ecosphere import CoppeliaEcosphere
from simulation.simulator.adapter.vrep.vrep_simulator_adapter import CoppeliaSimulatorAdapter

if __name__ == "__main__":

    vrep = CoppeliaSimulatorAdapter(CoppeliaEcosphere("robobo"))
    print(vrep.environment.robobo.simulator.client_id)
    print("finished")
    vrep._disconnect()

    evosphere = CoppeliaEvosphere()
    evosphere.evolve()
