from robobo.robot.robobo_simulation import RoboboSimulation
from simulation.vrep_simulator import VREPSimulator


class RoboboSimulationPrey(RoboboSimulation):

    def __init__(self, simulator: VREPSimulator, name="#0"):
        super(RoboboSimulationPrey, self).__init__(simulator, name)
