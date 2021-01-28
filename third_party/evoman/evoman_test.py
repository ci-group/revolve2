import threading
import time

from third_party.evoman.controller import SimpleNeuralNetwork
from third_party.evoman.evoman_environment import EvomanEnvironment, EvomanConfiguration


class Simulation:

    def __init__(self, environment: EvomanEnvironment):
        self.environment: EvomanEnvironment = environment

    # checks objective mode
    def test(self):
        player_controller = SimpleNeuralNetwork()

        self.run_single(player_controller)

    # runs game for a single enemy
    def run_single(self, player_controller, parameters=None):

        done, fitness, observation = self.environment.reset()
        if parameters is None:
            parameters = player_controller.random_weights()

        while not done:
            actions = player_controller.activate(observation, parameters)
            done, fitness, observation = self.environment.step(actions)
            self.environment.render()

        print(fitness)

    def close(self):
        self.environment.close()


def thread_worker():
    s = Simulation(EvomanEnvironment(EvomanConfiguration(headless=True)))
    s.test()


if __name__ == "__main__":
    print("start")
    start = time.time()
    threads = list()
    for index in range(5):
        x = threading.Thread(target=thread_worker)
        threads.append(x)
        x.start()

    for x in threads:
        x.join()

    end = time.time()
    print("end")
    print(end - start)
