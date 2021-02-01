import time

import numpy as np

from nca.core.actor.agent import Agent
from resources import vrep
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.robot import Robot
from simulation.simulation_measures import SimulationMeasures
from simulation.simulator.adapter.simulation_adapters import AgentSimulatorAdapter
from simulation.simulator.adapter.vrep.robobo.robobo_environment import RoboboEnvironment
from simulation.simulator.adapter.vrep.robobo.robobo_simulation import RoboboSimulation
from simulation.simulator.adapter.vrep.coppelia_simulator import CoppeliaSimulator


class VREPCommunicationError(Exception):
    pass


class CoppeliaSimulatorAdapter(AgentSimulatorAdapter):

    def __init__(self, ecosphere: Ecosphere):
        super().__init__(ecosphere)
        coppelia_path = "C:\\\"Program Files\"\\CoppeliaRobotics\\CoppeliaSimEdu"
        scene_path = "C:\\Users\\daan_\\Documents\\GitHub\\revolve2\\resources\\vrep\\scenes\\Robobo_Scene.ttt"
        self.environment = RoboboEnvironment(RoboboSimulation(CoppeliaSimulator(coppelia_path, scene_path, headless=True)))
        self.agent: Agent = None

    def _add_agent(self, agent: Agent):
        self.agent = agent

    def _remove_agent(self, agent: Agent):
        self.agent = None

    def _simulate(self) -> SimulationMeasures:
        done, fitness, observation = self.environment.reset()

        while not done:
            actions = self.agent.brain.activate(observation)
            done, fitness, observation = self.environment.step(actions)
            #self.environment.render()

        measures = SimulationMeasures()
        measures.fitness = np.linalg.norm(np.array([0.0, 0.0]) - np.array(fitness[:2]))

        return measures

    def _connect(self, address='127.0.0.1', port=19997):
        self.environment.robobo.simulator.connect(address, port)

    def _disconnect(self):
        self.environment.robobo.simulator.disconnect()

    def _pause_simulation(self):
        self.environment.robobo.simulator.pause_simulation()

    def _start_simulation(self):
        self.environment.robobo.simulator.play_simulation()

    def _stop_simulation(self):
        self.environment.robobo.simulator.stop_world()

    def get_simulation_time(self):
        self.environment.robobo.simulator.get_simulation_time()
