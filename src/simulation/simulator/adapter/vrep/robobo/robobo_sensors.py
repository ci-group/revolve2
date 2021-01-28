import math
from typing import Dict, List

import cv2
import numpy as np

from resources import vrep
from simulation.simulator.adapter.vrep.coppelia_simulation import CoppeliaSimulation


def _infrared_point_distance(point) -> float:
    return np.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2)


class RoboboSensors(object):
    """
    returns sensor readings: [backR, backC, backL, frontRR, frontR, frontC, frontL, frontLL]
    """

    def __init__(self, simulation: CoppeliaSimulation, name: str):
        self.sensors: Dict[str, object] = {}
        self.simulation: CoppeliaSimulation = simulation
        self.name = name

        self.frontal_camera = None

        self.object_handle_names = ['Ir_Back_L{}', 'Ir_Back_C{}', 'Ir_Back_R{}',
                                    'Ir_Front_LL{}', 'Ir_Front_L{}', 'Ir_Front_C{}', 'Ir_Front_R{}',  'Ir_Front_RR{}']
        # Unused 'Ir_Back_L_Floor{}', 'Ir_Back_R_Floor{}','Ir_Front_L_Floor{}''Ir_Front_R_Floor{}',

        self.sensor_keys = ['back_left', 'back_center', 'back_right',
                            'front_far_left', 'front_left', 'front_center', 'front_right', 'front_far_right']
        # Unused 'back_left_floor', 'back_right_floor', 'front_left_floor', 'front_right_floor',

        self._initialize()

    def _initialize(self):
        self.frontal_camera = self.simulation.get_object_handle('Frontal_Camera{}'.format(self.name))

        for sensor_key, object_handle_name in zip(self.sensor_keys, self.object_handle_names):
            self.sensors[sensor_key] = self.simulation.get_object_handle(object_handle_name.format(self.name))
            self.simulation.read_proximity_sensor(self.sensors[sensor_key], opmode=vrep.simx_opmode_streaming, ignore=True)

        self.simulation.get_vision_sensor_image(self.frontal_camera, vrep.simx_opmode_streaming, ignore=True)

    def read_camera(self):
        # get image
        resolution, image = self.simulation.get_vision_sensor_image(self.frontal_camera)

        # reshape image
        image = image[::-1]
        im_cv2 = np.array(image, dtype=np.uint8)
        im_cv2.resize([resolution[0], resolution[1], 3])
        im_cv2 = cv2.flip(im_cv2, 1)

        return im_cv2

    def read_infrared(self) -> List[float]:
        return [self._infrared_value(self.sensors[sensor_key] for sensor_key in self.sensors.keys())]

    def _infrared_value(self, object_handle) -> float:
        detection_state, detected_point, _, _ = self.simulation.read_proximity_sensor(object_handle)
        return _infrared_point_distance(detected_point) if detection_state else -math.inf
