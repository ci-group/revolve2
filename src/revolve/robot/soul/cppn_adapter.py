import numpy as np

import os

import neat
from shutil import copyfile

from nca.experiment_manager import ExperimentManager
from revolve.robot.brain.brain import Brain


class CPPNAdapter:

    def __init__(self, number_inputs: int, hidden_units: int, number_outputs: int):

        arguments = {"inputs": number_inputs, "hidden": hidden_units, "outputs": number_outputs}
        destination_path = self._replace_template(arguments)
        self.config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                             neat.DefaultSpeciesSet, neat.DefaultStagnation,
                             destination_path)

        self.model = self.config.genome_type('')
        self.model.fitness = 0
        self.model.configure_new(self.config.genome_config)
        self.network = neat.nn.FeedForwardNetwork.create(self.model, self.config)

    def _replace_template(self, arguments):
        path = ExperimentManager().folders.config_path
        template_path = os.path.join(path, "neuro_evolution/example_cppn_brain_template")
        destination_path = os.path.join(path, "neuro_evolution/example_cppn_brain")
        copyfile(template_path, destination_path)

        # Read in the file
        with open(destination_path, 'r') as file:
            filedata = file.read()

        # Replace the target string
        filedata = filedata.replace('@', str(arguments['hidden']))
        filedata = filedata.replace('$', str(arguments['inputs']))
        filedata = filedata.replace('&', str(arguments['outputs']))

        # Write the file out again
        with open(destination_path, 'w') as file:
            file.write(filedata)

        return destination_path

    def activate(self, inputs):
        return self.network.activate(inputs)

    def __repr__(self):
        return str(self.network)

    def classify(self, output, options):
        index = np.argmax(np.array(output))
        return options[index]


if __name__ == "__main__":
    brain = CPPNAdapter(2, 5, 10)

    a = 10
    b = 5
    brain.activate((a, b))
    print(brain)
