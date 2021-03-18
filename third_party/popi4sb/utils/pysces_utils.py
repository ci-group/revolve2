import numpy as np


def read_real_parameters(mod, params):
    real_params = {}

    for key in params:
        real_params[key] = getattr(mod, key)

    return real_params


# An auxiliary function that overwrites parameters values in the model
def overwrite_params(theta, mod, params, dic=False):
    for i in range(len(params)):
        if dic:
            setattr(mod, params[i], theta[params[i]])
        else:
            setattr(mod, params[i], theta[i])

    return mod


# An auxiliary function to remove fixed_species from parameters
def remove_fixed(parameters, fixed_species, compartment=True):
    parameters_list = list(parameters)
    fixed_species_list = list(fixed_species)

    for item in fixed_species_list:
        parameters_list.remove(item)

    if compartment:
        for item in parameters_list:
            if 'compartment' in item:
                parameters_list.remove(item)

    return tuple(parameters_list)


def generate_data(mod, params, sim_start=None, sim_end=None, sim_points=None, theta=None, noise=0.):
    if theta is not None:
        mod = overwrite_params(theta, mod, params)

    if sim_start is not None:
        mod.sim_start = sim_start
        mod.sim_end = sim_end
        mod.sim_points = sim_points

    mod.Simulate()

    data = mod.data_sim.getSpecies()

    t = data[:, [0]].T

    x = data[:, 1:].T

    if noise > 0.:
        norm = np.expand_dims(np.max(x, 1), 1)
        mean = x / norm
        x = (mean + np.random.randn(mean.shape[0], mean.shape[1]) * (mean * noise)) * norm

    return x, t