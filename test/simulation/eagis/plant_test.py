
import unittest
import matplotlib.pyplot as plt

from revolve2.nca.core.ecology.population import Population
from revolve2.simulation.AEGIS.scape_environment import ScapeEnvironment

import imageio


class AEGISPlants(unittest.TestCase):

    def test_plants(self):

        scape_env = ScapeEnvironment(50, 50, 10, Population([]))

        filenames = []
        for step in range(300):
            scape_env.step([])
            filenames.append(scape_env.render())

        images = []
        for filename in filenames:
            images.append(imageio.imread(filename))

        imageio.mimsave("../../../log/aegis/animation.gif", images)

        plt.close()
        plt.plot(scape_env.history['flora_count'])
        plt.plot(scape_env.history['animal_count'])
        plt.legend(labels=["flora", 'animals'])
        plt.show()
