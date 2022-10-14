from typing import List
import numpy as np
import random


class SensorData:
    _roations: List[float]
    _touch_states: List[int]

    def __init__(self,
        rotations: List[float] = None,
        touch_states: List[float] = None
        ) -> None:

        self._roations = rotations
        self._touch_states = touch_states

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


