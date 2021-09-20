from robobo.robot.robobo_simulation import RoboboSimulation
from revolve2.simulation.vrep_simulator import VREPSimulator


class RoboboSimulationPrey(RoboboSimulation):

    def __init__(self, simulator: VREPSimulator, name="#0"):
        super(RoboboSimulationPrey, self).__init__(simulator, name)
