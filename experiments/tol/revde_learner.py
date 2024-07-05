import numpy as np
from numpy.typing import NDArray
from .revde import recombination


class DifferentialEvolution:
    gen: int = 0
    mean_first: float
    f_best_so_far: np.float64 = np.inf

    def __init__(self, x0: NDArray, population_size: int):
        self.population_size = population_size

        self.x_current = x0
        self.x_previous = np.empty(shape=x0.shape, dtype=np.float64)
        self.f = np.empty(shape=x0.shape[0], dtype=np.float64)

    def add_eval(self, fitness):
        current_genome, self.x_current = self.x_current[:1], self.x_current[1:]
        self.x_previous = np.vstack((self.x_previous, current_genome))
        self.f = np.append(self.f, fitness)
        if self.x_current.shape[0] == 0:
            return True
        return False

    def new_pop(self):
        self.mean_first = np.mean(self.f) if self.gen == 0 else self.mean_first

        x, f = self._select(self.x_current, self.f, population_size=self.population_size)
        f_min = np.min(f)
        self.f_best_so_far = f_min if f_min < self.f_best_so_far else self.f_best_so_far

        self.x_current = recombination(x, bounds=(0, 1), F=0.9, CR=0.5)
        self.x_previous = x
        self.f = f

        self.gen += 1

    @staticmethod
    def _select(x, f, population_size: int):
        indices = np.argsort(f)
        x_new, f_new = x[indices], f[indices]

        return x_new[0:population_size], f_new[0:population_size]