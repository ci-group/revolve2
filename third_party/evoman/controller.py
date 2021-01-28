################################
# EvoMan FrameWork - V1.0 2016 #
# Author: Karine Miras         #
# karine.smiras@gmail.com      #
################################

import numpy as np


def sigmoid_activation(x):
    return 1./(1.+np.exp(-x))


def binary_activation(x):
    return x > 0.5


# implements controller structure for enemy
def normalize_minmax(inputs):
    max_value = max(inputs)
    min_value = min(inputs)
    return (inputs - min_value) / float(max_value - min_value)


class SimpleNeuralNetwork(object):

    def __init__(self, number_inputs: int = 20, hidden_units: int = 10, number_actions: int = 5,
                 activation_function=sigmoid_activation, classification_function=binary_activation,
                 normalization_function=normalize_minmax):
        self.number_inputs = number_inputs
        self.number_hidden_units = hidden_units
        self.number_actions = number_actions

        self.activation_function = activation_function
        self.classification_function = classification_function
        self.normalization_function = normalization_function

    def random_weights(self):
        return np.random.normal(0, 1, size=[self.number_hidden_units + (self.number_inputs * self.number_hidden_units) + \
               self.number_actions + (self.number_hidden_units * self.number_actions)])

    def activate(self, inputs, parameters):

        inputs = self.normalization_function(inputs)

        if self.number_hidden_units > 0:
            outputs = self.activate_hidden(inputs, parameters)
        else:
            outputs = self.activate_inputs(inputs, parameters)

        return [self.classification_function(output) for output in outputs]

    def activate_hidden(self, inputs, parameters):
        assert len(parameters) == self.number_hidden_units + (len(inputs) * self.number_hidden_units) + \
               self.number_actions + (self.number_hidden_units * self.number_actions)
        # Biases for the n hidden neurons
        bias1 = parameters[:self.number_hidden_units].reshape(1, self.number_hidden_units)
        # Weights for the connections from the inputs to the hidden nodes
        weights1_slice = len(inputs) * self.number_hidden_units + self.number_hidden_units
        weights1 = parameters[self.number_hidden_units:weights1_slice].reshape(
            (len(inputs), self.number_hidden_units))

        # Outputs activation first layer.
        output1 = self.activation_function(inputs.dot(weights1) + bias1)

        # Preparing the weights and biases from the controller of layer 2
        bias2 = parameters[weights1_slice:weights1_slice + self.number_actions].reshape(1, self.number_actions)
        weights2 = parameters[weights1_slice + self.number_actions:].reshape(
            (self.number_hidden_units, self.number_actions))

        # Outputting activated second layer. Each entry in the output is an action
        return self.activation_function(output1.dot(weights2) + bias2)[0]

    def activate_inputs(self, inputs, parameters):
        assert len(parameters) == self.number_hidden_units + (len(inputs) * self.number_actions)

        bias = parameters[:self.number_actions].reshape(1, self.number_actions)
        weights = parameters[self.number_actions:].reshape((len(inputs), self.number_actions))

        return self.activation_function(inputs.dot(weights) + bias)[0]
