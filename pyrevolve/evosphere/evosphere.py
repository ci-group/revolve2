from typing import List

from pyrevolve.agent.birth_clinic import BirthClinic
from pyrevolve.evosphere.environment import Environment
from pyrevolve.evosphere.performance_measures import PerformanceMeasures


class EvoSphere:

    def __init__(self):
        self.birth_clinic: BirthClinic = BirthClinic()
        self.environments: List[Environment] = []
        self.measures: PerformanceMeasures()
