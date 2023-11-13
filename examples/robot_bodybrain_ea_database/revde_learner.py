import numpy as np
from numpy.typing import NDArray

from revde.algorithms.recombination import DifferentialRecombination
from revde.algorithms.selections import SelectBest


class DifferentialEvolution:
    gen: int = 0
    mean_first: float
    f_best_so_far: np.float64 = np.inf

    def __init__(self, x0: NDArray, population_size: int, bounds: tuple[int, int] = (0, 1)):
        self.population_size = population_size

        self.x_current = x0
        self.x_previous = np.empty(shape=x0.shape, dtype=np.float64)
        self.f = np.empty(shape=x0.shape[0], dtype=np.float64)

        self.fd_best_so_far = []

        params = {
            "F": 0.9,
            "CR": 0.5,
        }

        self.differential = DifferentialRecombination(bounds=bounds, params=params)
        self.selection = SelectBest()

    def add_eval(self, fitness):
        current_genome, self.x_current = self.x_current[:1], self.x_current[1:]
        self.x_previous = np.vstack((self.x_previous, current_genome))
        self.f = np.append(self.f, fitness)
        if self.x_current.shape[0] == 0:
            return True
        return False

    def new_pop(self):
        self.mean_first = np.mean(self.f) if self.gen == 0 else self.mean_first

        x, f = self.selection.select(self.x_current, self.f, population_size=self.population_size)

        f_min = np.min(f)
        self.f_best_so_far = f_min if f_min < self.f_best_so_far else self.f_best_so_far

        self.x_current = self.differential.recombination(x)
        self.x_previous = x
        self.f = f

        self.gen += 1
        # print(f"New population, gen: {self.gen} \t | \t {time.clock()}")
