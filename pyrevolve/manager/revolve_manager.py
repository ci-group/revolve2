from pyrevolve.evosphere.evosphere import EvoSphere
from pyrevolve.experiment.experiment_manager import ExperimentManager


def run():
    experiment_manager = ExperimentManager.Instance()

    evosphere = EvoSphere()


if __name__ == "__main__":
    run()