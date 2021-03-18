import numpy as np

from third_party.popi4sb.algorithms.evolutionary_model import EvolutionaryModel


class EvolutionaryStrategies(EvolutionaryModel):
    def __init__(self, config_method, config_model, calculate_fitness, c: float = 0.817):
        super().__init__(config_method, config_model, calculate_fitness)
        print('Initialized ES.')

        self.sigma = config_method['std']
        self.c = c

    def _proposal(self, theta, E = None):
        noise = self.sigma * np.random.randn(theta.shape[0], theta.shape[1])
        return super()._propose_theta(theta, noise, E)

    def step(self, theta, E_old):
        # (1. Generate)
        theta_new = super()._generate(theta, E_old)

        # (2. Evaluate)
        E_new = self.calculate_fitness(theta_new)

        # (3. Select)
        m = (E_new < E_old) * 1.

        if np.mean(m) < 0.2:
            self.sigma = self.sigma * self.c
        elif np.mean(m) > 0.2:
            self.sigma = self.sigma / self.c

        return m * theta_new + (1. - m) * theta, m * E_new + (1. - m) * E_old
