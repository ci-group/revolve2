import time
from abc import abstractmethod, ABC
import numpy as np


class EvolutionaryModel(ABC):

    def __init__(self, config_method, config_model, calculate_fitness, parameter_size):
        self.config_method = config_method
        self.config_model = config_model

        self.calculate_fitness = calculate_fitness

        self.parameter_size = parameter_size

        self.indices = self.config_model['indices']

        self.theta = None
        self.E = None

    def initialize(self, theta, E):
        self.theta = theta
        self.E = E

    @abstractmethod
    def _proposal(self, theta, E = None):
        pass

    def step(self, theta, E_old):
        theta_new = self._generate(theta, E_old)

        # (2. Evaluate)
        E_new = self.calculate_fitness(theta_new, self.config_model, self.config_method)

        # (3. Select)
        theta_cat = np.concatenate((theta, theta_new), 0)
        E_cat = np.concatenate((E_old, E_new), 0)

        index = np.argsort(E_cat.squeeze())

        return theta_cat[index[:theta.shape[0]], :], E_cat[index[:theta.shape[0]], :]

    def _generate(self, theta, E_old):
        return np.clip(self._proposal(theta, E_old), a_min=self.config_method['clip_min'],
                       a_max=self.config_method['clip_max'])

    def _propose_theta(self, theta, de_noise, E):
        if self.config_method['best']:
            tht = theta[[np.argmin(E)]]
        else:
            tht = theta

        theta_new = tht + de_noise

        p_1 = np.random.binomial(1, self.config_method['CR'], tht.shape)
        return p_1 * theta_new + (1. - p_1) * tht


class EvolutionaryExperiment:

    def __init__(self, model: EvolutionaryModel, config_method, experiment_path):
        self.model = model
        self.model.initialize()

        self.config_method = config_method

        self.experiment_path = experiment_path

        self.theta = None
        self.E = None

    def run(self):
        best_E = [np.min(self.E)]

        all_E = self.E
        all_theta = self.theta

        clock_start = time.time()
        print('START ~~~~~~>')
        g = self.model.config_method.config['generations']
        for i in range(g):
            print(f'========> Generation {i + 1}/{g}')
            theta, E = self.model.step(self.theta, self.E)
            if np.min(E) < best_E[-1]:
                best_E.append(np.min(E))
            else:
                best_E.append(best_E[-1])

            all_theta = np.concatenate((all_theta, theta), 0)
            all_E = np.concatenate((all_E, E), 0)

            self._save_progress(all_theta, all_E, best_E)
            # early stopping
            if i > self.config_method.config['patience']:
                if best_E[-self.config_method.config['patience']] == best_E[-1]:
                    break

        print('~~~~~~> END')
        clock_stop = time.time()
        print('Time elapsed: {}'.format(clock_stop - clock_start))
        self._save_result(clock_start, clock_stop)

    def _save_progress(self, all_theta, all_E, best_E):
        np.save(self.experiment_path + 'all_theta.npy', all_theta)
        np.save(self.experiment_path + 'all_E.npy', all_E)
        np.save(self.experiment_path + 'best_E.npy', np.asarray(best_E))

    def _save_result(self, clock_start, clock_stop):
        np.save(self.experiment_path + 'time.npy', np.asarray(clock_stop - clock_start))