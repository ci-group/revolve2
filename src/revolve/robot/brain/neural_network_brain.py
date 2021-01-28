from typing import List

import numpy as np

from nca.core.evolution.conditions.initialization import ValuedInitialization
from nca.core.genome.operators.initialization import GaussianInitialization
from nca.core.genome.representations.valued_representation import ValuedRepresentation
from revolve.robot.brain.brain import Brain, AgentBrain


def sigmoid_activation(x):
    return 1./(1.+np.exp(-x))


def binary_activation(x):
    return x > 0.5


# implements controller structure for enemy
def normalize_minmax(inputs):
    max_value = max(inputs)
    min_value = min(inputs)
    return (inputs - min_value) / float(max_value - min_value)


class NeuralNetworkBrain(AgentBrain):

    def __init__(self, number_inputs: int = 20, hidden_units: int = 10, number_actions: int = 5,
                 activation_function=sigmoid_activation, classification_function=binary_activation,
                 normalization_function=normalize_minmax):
        self.number_inputs = number_inputs
        self.number_hidden_units = hidden_units
        self.number_actions = number_actions

        self.activation_function = activation_function
        self.classification_function = classification_function
        self.normalization_function = normalization_function

        self.parameters: List[float] = []

    def representation(self):
        return ValuedRepresentation(GaussianInitialization())

    def size(self):
        return self.number_hidden_units + (self.number_inputs * self.number_hidden_units) + \
               self.number_actions + (self.number_hidden_units * self.number_actions)

    def update(self, parameters):
        self.parameters = np.array(parameters)

    def activate(self, inputs):

        inputs = self.normalization_function(inputs)

        if self.number_hidden_units > 0:
            outputs = self.activate_hidden(inputs, self.parameters)
        else:
            outputs = self.activate_inputs(inputs, self.parameters)

        return [self.classification_function(output) for output in outputs]

    def activate_hidden(self, inputs, parameters):
        assert len(parameters) == self.size()

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
        assert len(parameters) == self.size()

        bias = parameters[:self.number_actions].reshape(1, self.number_actions)
        weights = parameters[self.number_actions:].reshape((len(inputs), self.number_actions))

        return self.activation_function(inputs.dot(weights) + bias)[0]
