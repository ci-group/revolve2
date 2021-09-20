from __future__ import unicode_literals, print_function, absolute_import, division, generators, nested_scopes

import math
import time

import numpy as np

from resources import vrep
from resources.vrep import VrepApiError
from revolve2.simulation.simulator.adapter.vrep.coppelia_simulator import CoppeliaSimulator


class CoppeliaSimulation:

    def __init__(self, simulator: CoppeliaSimulator):
        self.simulator: CoppeliaSimulator = simulator

    def sleep(self, seconds):
        duration = seconds * 1000
        start_time = self.simulator.get_simulation_time()
        while self.simulator.get_simulation_time() - start_time < duration:
            pass

    def get_object_handle(self, object_name, operation_mode=vrep.simx_opmode_blocking):
        print(object_name)
        return vrep.unwrap_vrep(vrep.simxGetObjectHandle(self.simulator.client_id, object_name, operation_mode))

    # position and velocities, and orientation

    def get_object_position(self, object_handle, operation_mode=vrep.simx_opmode_blocking):
        self.simulator.wait_for_ping()
        return vrep.unwrap_vrep(
            vrep.simxGetObjectPosition(self.simulator.client_id, object_handle, -1, operation_mode)
        )

    def set_object_position(self, object_handle, position, opmode=vrep.simx_opmode_oneshot):
        value = vrep.unwrap_vrep(vrep.simxSetObjectPosition(self.simulator.client_id, object_handle, -1, position, opmode))
        self.simulator.wait_for_ping()
        return value

    def set_object_orientation(self, object_handle, orientation, opmode=vrep.simx_opmode_oneshot):
        value = vrep.unwrap_vrep(vrep.simxSetObjectOrientation(self.simulator.client_id, object_handle, -1, orientation, opmode))
        self.simulator.wait_for_ping()
        return value

    def set_joint_position(self, handle, position, opmode=vrep.simx_opmode_oneshot):
        value = vrep.unwrap_vrep(vrep.simxSetJointTargetPosition(self.simulator.client_id, handle, position, opmode))
        self.simulator.wait_for_ping()
        return value

    def set_joint_velocity(self, object_handle, target_velocity, operation_mode=vrep.simx_opmode_oneshot):
        value = vrep.unwrap_vrep(vrep.simxSetJointTargetVelocity(self.simulator.client_id, object_handle,
                                                                 target_velocity, operation_mode))
        self.simulator.wait_for_ping()
        return value

    def get_orientation(self, object_handle):
        self.simulator.wait_for_ping()
        euler_angles = vrep.unwrap_vrep(vrep.simxGetObjectOrientation(
            self.simulator.client_id, object_handle, -1, vrep.simx_opmode_blocking)
        )
        print(euler_angles)
        yaw, pitch, _ = euler_angles

        orientation = math.copysign(1, yaw) * math.pi / 2 + pitch
        if orientation < 0:
            orientation = -(math.pi + orientation)
        return orientation

    # Infrared sensors
    def read_proximity_sensor(self, proximity_handle, opmode=vrep.simx_opmode_buffer, ignore=False):
        self.simulator.wait_for_ping()
        try:
            print(self.simulator.client_id, proximity_handle, opmode)
            return vrep.unwrap_vrep(vrep.simxReadProximitySensor(self.simulator.client_id, proximity_handle, opmode))
        except VrepApiError as error:
            if error.ret_code is not vrep.simx_return_novalue_flag and not ignore:
                raise

    # Vision sensor
    def get_vision_sensor_image(self, camera_handle, opmode=vrep.simx_opmode_buffer, a=0, ignore=False):
        self.simulator.wait_for_ping()
        try:
            return vrep.unwrap_vrep(vrep.simxGetVisionSensorImage(self.simulator.client_id, camera_handle, a, opmode))
        except VrepApiError as error:
            if error.ret_code is not vrep.simx_return_novalue_flag and not ignore:
                raise

    def set_speed_position(self, object_handle, target_position, manipulation_speed, samples=25):
        starting_position = self.get_object_position(object_handle)
        positional_intervals = np.linspace(starting_position, target_position, samples)
        for current_position in positional_intervals:
            self.set_joint_position(object_handle, current_position)
            time.sleep(manipulation_speed / samples)
