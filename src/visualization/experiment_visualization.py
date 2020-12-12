from visualization.visualization import StatisticsVisualization


class ExperimentVisualization(StatisticsVisualization):

    def __init__(self, experiment: Experiment):
        super().__init__()
        self.experiment = experiment


class SpeciationVisualization(ExperimentVisualization):
    pass


class GenerationalFitnessVisualization(ExperimentVisualization):
    pass
