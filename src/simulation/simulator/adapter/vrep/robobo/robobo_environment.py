import math
from typing import List

from simulation.simulator.adapter.vrep.robobo.robobo_simulation import RoboboSimulation


def angle_difference(x, y):
    return math.atan2(math.sin(x - y), math.cos(x - y))


class RoboboEnvironment:
    linear_velocity = 1
    wheel_distance = 0.145
    wheel_radius = 0.03

    def __init__(self, simulation: RoboboSimulation, enable_camera: bool = False):
        self.robobo: RoboboSimulation = simulation
        self.enable_camera: bool = enable_camera

        self.done: bool = False
        self.total_time: int = 100
        self.current_time: int = 0

    def reset(self):
        self.robobo.reset()
        self.done = False
        return self.done, self.robobo.state(), self.get_sensors()

    def differential_drive(self, direction):
        robot_orientation = self.robobo.get_orientation(self.robobo.robot)
        delta_orientation = angle_difference(direction, robot_orientation)

        velocity_right = self.linear_velocity + self.wheel_distance * delta_orientation
        velocity_left = self.linear_velocity - self.wheel_distance * delta_orientation
        speed_right = velocity_right / self.wheel_radius
        speed_left = velocity_left / self.wheel_radius

        self.robobo.actuators.drive(speed_left, speed_right)

    def get_sensors(self) -> List[float]:
        if self.enable_camera:
            return self.robobo.sensors.read_camera()
        else:
            return self.robobo.sensors.read_infrared()

    def step(self, actions):
        """
        Two possibilities:
        1: Direct drive
         - Argument 1: Left speed,
         - Argument 2: Right speed
        2: Differential drive
         - Argument 1: Drive angle
        """
        if self.current_time >= self.total_time:
            self.done = True
            return self.done, self.robobo.state(), self.get_sensors()

        if len(actions) == 1:
            self.robobo.actuators.differential_drive(direction=actions[0])
        else:
            self.robobo.actuators.drive(speed_left=actions[0], speed_right=actions[1])

        self.current_time += 1
        return self.done, self.robobo.state(), self.get_sensors()
