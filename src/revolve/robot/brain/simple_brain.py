from abc import ABC

from revolve.robot.brain.brain import Brain


class SimpleOscillatorBrain(Brain):

    def __init__(self, number_of_oscillators=5):
        self.genotype()
    pass

