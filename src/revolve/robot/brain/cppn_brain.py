from revolve.robot.brain.brain import Brain
from revolve.robot.soul.cppn_adapter import CPPNAdapter


class CPPNBrain(Brain):

    def __init__(self, number_inputs: int = 2, hidden_units: int = 10, number_outputs: int = 5):
        self.cppn = CPPNAdapter(number_inputs, hidden_units, number_outputs)

    def activate(self, inputs):
        self.cppn.activate(inputs)


if __name__ == "__main__":
    print(CPPNBrain().cppn.model)
