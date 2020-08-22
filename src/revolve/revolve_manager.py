from nca.experiment_manager import ExperimentManager
from revolve.evosphere.evosphere import RevolveEvosphere

experiment_manager = ExperimentManager()


def run_regular():
    evosphere = RevolveEvosphere()
    evosphere.evolve()


if __name__ == "__main__":
    run_regular()