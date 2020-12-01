from visualization.visualization import StatisticsVisualization, Visualization


class IndividualVisualization(StatisticsVisualization):

    def __init__(self, individual):
        super().__init__()
        self.individual = individual


class BrainVisualization(Visualization, IndividualVisualization):
    pass


class BodyVisualization(Visualization, IndividualVisualization):
    pass
