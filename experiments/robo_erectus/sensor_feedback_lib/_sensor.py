from typing import List
import numpy as np
import random


class SensorData:
    _roations: List[float]
    _touch_states: List[int]
    _num_cpgs: int

    def __init__(self,
        rotations: List[float] = None,
        touch_states: List[float] = None
        ) -> None:

        self._roations = rotations
        self._touch_states = touch_states
    
    def set_touch_states(self, groundcontacts, geom_bodyids, num_cpgs):
        touch_states = []
        for geom_id in groundcontacts:
            # the actual number of bodies is less 1 than model.nbody(mujoco.MjModel) 
            touch_states.append(geom_bodyids[geom_id]-1)
        # n_body = n_joint + 1
        assert len(set(touch_states)) <= num_cpgs + 1
        self._touch_states = list(set(touch_states))
        self._num_cpgs = num_cpgs

    def get_sensor_inputs(self):
        if self._num_cpgs == 0:
            return []
        sensor_inputs = np.zeros(self._num_cpgs)
        for module_id in self._touch_states:
            # n_body = n_joint + 1, if module i touch ground, then joint i-1 need to adjust angle.
            sensor_inputs[int(module_id-1)] = 1
        return sensor_inputs

    @staticmethod
    def get_random_data(num_cpgs):
        # to generate random numbers belong to [a, b], use a + (b-a)*np.random.random()
        # here a=-1, b=1
        _roations = []
        _touch_states = []
        if num_cpgs==0:
            return _roations, _touch_states
            
        _roations = -1 + 2*np.random.random(num_cpgs)
        indexs = [ i for i in range(num_cpgs)]
        _touch_states = np.zeros(num_cpgs)
        num_select_sample = random.sample(indexs, 1)[0]
        indexs_sample = random.sample(indexs, num_select_sample)
        for index in indexs_sample:
            _touch_states[index] = 1
        return _roations, _touch_states


