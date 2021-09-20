import time
from math import pi
from typing import List

from resources import vrep
from revolve2.simulation.simulator.adapter.vrep.robobo.robobo_actuators import RoboboActuators
from revolve2.simulation.simulator.adapter.vrep.robobo.robobo_sensors import RoboboSensors
from revolve2.simulation.simulator.adapter.vrep.coppelia_simulator import CoppeliaSimulator
from revolve2.simulation.simulator.adapter.vrep.coppelia_simulation import CoppeliaSimulation


class RoboboSimulation(CoppeliaSimulation):

    def __init__(self, simulator: CoppeliaSimulator, name: str = ""):
        super().__init__(simulator)

        self.name: str = name
        self.robot = None

        self.starting_state: List[float] = []

        self.initialize()

    def initialize(self):
        self.simulator.connect()
        get_handles_timeout = 120.0

        start_time = time.time()
        while time.time() - start_time < get_handles_timeout:
            try:
                self._initialize_handles()
                self.starting_state = self.state()
                return self
            except vrep.VrepApiError as _e:
                print("Handle initialization failed, retrying.")
                time.sleep(1)

        return False

    def _initialize_handles(self):
        self.robot = self.get_object_handle('Robobo{}'.format(self.name))
        self.get_object_position(self.robot)

        self.actuators = RoboboActuators(super(), self.name)

        self.sensors = RoboboSensors(super(), self.name)

    def state(self):
        position = self.get_object_position(self.robot)
        orientation = self.get_orientation(self.robot)

        return [position[0], position[1], orientation]

    def spin(self):
        raise NotImplementedError("Not implemeted yet")

    def collected_food(self):
        ints, floats, strings, buffer = vrep.unwrap_vrep(
            vrep.simxCallScriptFunction(self.simulator.client_id, "Food", vrep.sim_scripttype_childscript,
                                        "remote_get_collected_food",
                                        [], [], [], bytearray(), vrep.simx_opmode_blocking)
        )
        return ints[0]

    def reset(self):
        self.set_object_position(self.robot, self.starting_state[:2])
        self.set_object_orientation(self.robot, [-pi/2, -pi/2, -pi/2])
        self.actuators.reset()
