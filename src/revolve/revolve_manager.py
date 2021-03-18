from nca.experiment_manager import ExperimentManager
from revolve.evosphere.evosphere import RobotEvosphere

experiment_manager = ExperimentManager()


def run_regular():
    evosphere = RobotEvosphere()
    evosphere.evolve()


if __name__ == "__main__":
    run_regular()
