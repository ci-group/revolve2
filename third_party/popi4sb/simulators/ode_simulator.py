import json
import os
import pickle

import numpy as np
import pysces
from scipy import stats

from third_party.popi4sb.utils.dictionary_tools import dict_to_array
from third_party.popi4sb.utils.pysces_utils import overwrite_params, remove_fixed, generate_data, read_real_parameters


class PySCESSimulatorODE:

    def __init__(self, config_model, config_method, config_solver, dir_data, file_data, dir_model, experiment_path):
        self.mod = None
        self.params = None
        self.x_obs = None

        self.config_model = config_model
        self.config_method = config_method
        self.config_solver = config_solver

        self.dir_data = dir_data
        self.file_data = file_data
        self.dir_model = dir_model

        self.experiment_path = experiment_path

        self.dist_name = self.config_method['dist_name']
        self.dist = stats.norm

        self.initialize()

    def initialize(self):
        # Load PySCES model
        mod = pysces.model(self.config_model.config['mod_name'], dir=self.dir_model)

        # Solver settings
        mod.__settings__["mode_sim_max_iter"] = self.config_solver.config["mode_sim_max_iter"]
        mod.__settings__['lsoda_atol'] = self.config_solver.config['lsoda_atol']
        mod.__settings__['lsoda_rtol'] = self.config_solver.config['lsoda_rtol']
        mod.__settings__['lsoda_mxordn'] = self.config_solver.config['lsoda_mxordn']
        mod.__settings__['lsoda_mxords'] = self.config_solver.config['lsoda_mxords']
        mod.__settings__['lsoda_mxstep'] = self.config_solver.config['lsoda_mxstep']

        # Remove fixed_species from params. We do it only once.
        params = remove_fixed(mod.parameters, mod.fixed_species, compartment=self.config_model.config['compartment'])

        if self.dir_data is not None:
            self.config_model.config['real_data'] = True
            mod.sim_start = self.config_model.config['sim_start']
            mod.sim_end = self.config_model.config['sim_end']
            mod.sim_points = self.config_model.config['sim_points']
            self.x_obs = np.load(os.path.join(self.dir_data, self.file_data))
        else:
            self.config_model.config['real_data'] = False
            self.x_obs, t = generate_data(mod, params, sim_start=self.config_model.config['sim_start'],
                                     sim_end=self.config_model.config['sim_end'],
                                     sim_points=self.config_model.config['sim_points'], noise=self.config_model.config['noise'])

            real_params = read_real_parameters(mod, params)
            real_params_array = dict_to_array(real_params, params)

            np.save(self.experiment_path + 'x_obs.npy', self.x_obs)
            np.save(self.experiment_path + 't.npy', t)
            np.save(self.experiment_path + 'real_params_array.npy', real_params_array)

            json.dump(real_params, open(self.experiment_path + 'real_params.json', "w"))
            json.dump(params, open(self.experiment_path + 'params.json', "w"))

        pickle.dump(mod, open(self.experiment_path + 'mod.pkl', "wb"))

        pickle.dump(self.config_method.config, open(self.experiment_path + 'config_method.pkl', "wb"))
        pickle.dump(self.config_model.config, open(self.experiment_path + 'config_model.pkl', "wb"))

    def simulation(self, theta):
        # overwrite parameters values of the model
        mod = overwrite_params(theta, self.mod, self.params)
        # run simulation
        mod.Simulate()

        # get synthetic data
        sim_data = mod.data_sim.getSpecies()

        return sim_data[:, 1:].T

    def fitness(self, x, theta):
        S_sim = self.simulation(theta)
        if self.config_model['real_data']:
            S_obs = x / np.expand_dims(np.max(x, 1), 1)
            S_sim = S_sim[self.config_model['indices']] / np.expand_dims(np.max(x, 1), axis=1)
        else:
            S_obs = x[self.config_model['indices']] / np.expand_dims(np.max(x[self.config_model['indices']], 1), axis=1)
            S_sim = S_sim[self.config_model['indices']] / np.expand_dims(np.max(x[self.config_model['indices']], 1), axis=1)

        # Fitness function defined as E(x) = log p(x)
        if np.isnan(S_sim).any():
            fitness = np.asarray([[10000.]])
        else:
            if self.config_method['dist_name'] == 'norm':
                difference = self.dist.logpdf(S_obs, loc=S_sim, scale=self.config_method['scale'])
            elif self.config_method['dist_name'] == 'abs':
                difference = -np.abs(S_obs - S_sim)
            else:
                raise ValueError('Wrong distribution name!')

            log_pdf = np.sum(difference, 1, keepdims=True)
            fitness = -np.sum(log_pdf, 0, keepdims=True)

        return fitness

    def calculate_fitness(self, x, theta):
        # Calculating fitness for a batch
        E_array = np.zeros((theta.shape[0], 1))

        for i in range(theta.shape[0]):
            E_array[i] = self.fitness(x, theta[i])

        return E_array
