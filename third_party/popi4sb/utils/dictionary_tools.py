import numpy as np


def dict_to_array(dic, params):
    array = []

    for key in params:
        array.append(dic[key])

    return np.asarray(array)


def array_to_dic(theta, params):
    result = {}

    for i in range(len(params)):
        result[params[i]] = theta[i]

    return result