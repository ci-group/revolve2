"""Evaluation functions."""

import numpy as np
import numpy.typing as npt


def evaluate_network(
    params: npt.NDArray[np.float_], inputs: npt.NDArray[np.float_]
) -> np.float_:
    """
    Pass two inputs through a fully connected relu network.

    :param params: The parameters to evaluate.
    :param inputs: Inputs for network. 2x1 floats.
    :returns: The output of the network.
    """
    # First layer
    n0 = np.maximum(0, np.dot(params[:2], inputs) + params[2])
    n1 = np.maximum(0, np.dot(params[3:5], inputs) + params[5])

    # Second layer
    output: np.float_ = np.maximum(0, n0 * params[6] + n1 * params[7] + params[8])

    return output


def evaluate(parameters: npt.NDArray[np.float_]) -> float:
    """
    Measure one set of parameters.

    :param parameters: The parameters to measure.
    :returns: Negative sum of squared errors and each individual error. 5x1 floats.
    """
    # Define all possible inputs for xor and the expected outputs
    inputs = np.array([[0, 0], [1, 0], [0, 1], [1, 1]])
    expected_outputs = np.array([0, 1, 1, 0])

    # Evaluate the provided network parameters
    outputs = np.array([evaluate_network(parameters, input) for input in inputs])

    # Calculate the difference between the network outputs and the expect outputs
    errors = outputs - expected_outputs

    # Return the sum of squared errors.
    # 0 would be an optimizal result.
    # We invert so we can maximize the fitness instead of minimize.
    # Finally we convert from a numpy float_ type to the python float type. This is not really important.
    return float(-np.sum(errors**2))
