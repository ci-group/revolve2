import math
import statistics
from typing import Dict, List

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from networkx.drawing.nx_pydot import graphviz_layout
from revolve2.analysis.statistics import Statistics
from revolve2.nca.core.actor.individual import Individual
from revolve2.nca.core.actor.measures import (BehavioralMeasurement,
                                              EpigeneticMeasurement,
                                              Measurement,
                                              MorphologicalMeasurement)


class Metrics:
    def __init__(self):
        pass


class PhylogenyMetrics:

    def __init__(self, generational_history: List[Dict[int, Individual]]):
        self.generational_history: List[Dict[int, Individual]] = generational_history
        self.complete_history: Dict[int, Individual] = self.combine_historical_logs()
        self.explored = None

    def combine_historical_logs(self) -> Dict[int, Individual]:
        complete_history: Dict[int, Individual] = {}
        for generational_history in self.generational_history:
            for individual in generational_history.values():
                complete_history[individual.id] = individual
        return complete_history

    def generational_statistics(self):
        s = Statistics()
        ages = []
        for individual in self.complete_history.values():
            ages.append(individual.age.generations)
        s.log_values(ages)
        print(s)

    def generational_depth(self, individual: Individual, depth=0, max_depth=math.inf):
        if depth == 0:
            self.explored = []

        parent_depth: Dict[int, List[int]] = {}

        if individual.id in self.explored:
            return {}

        if individual.parent_ids is None or depth == max_depth:
            return {individual.id: [depth]}

        self.explored.append(individual.id)

        for parent_id in individual.parent_ids:
            elements = self.generational_depth(self.complete_history[parent_id], depth+1, max_depth)
            for key in elements:
                if key in parent_depth:
                    parent_depth[key].extend(elements[key])
                else:
                    parent_depth[key] = elements[key]

        return parent_depth

    def graph(self):
        G = nx.Graph()

        for key in self.complete_history.keys():
            individual = self.complete_history[key]
            G.add_node(individual.id)

        for key in self.complete_history.keys():
            individual = self.complete_history[key]
            if individual.parent_ids is not None:
                for parent_id in individual.parent_ids:
                    G.add_edge(individual.id, parent_id)

        nx.draw(G, node_size=25, node_color=range(len(self.complete_history)), cmap=plt.cm.viridis, pos=graphviz_layout(G))
        plt.show()
        #app = Viewer(G)
        #app.mainloop()

    def heritability(self, steps: int = 10):
        correlations = []
        for index, generation in enumerate(self.generational_history):
            if index == 0:
                continue
            points_parents = []
            points_children = []
            for individual in generation.values():
                if len(individual.parent_ids) > 0:
                    parent_performance = []
                    for parent_id in individual.parent_ids:
                        parent_performance.append(self.complete_history[parent_id].fitness.value())
                    average_parent_performance = statistics.mean(parent_performance)
                    max_parent_performance = min(parent_performance)
                    points_parents.append(average_parent_performance)
                    points_children.append(individual.fitness.value())

            correlation = np.corrcoef(points_parents, points_children)[1, 0]
            correlations.append(correlation)
            if index % steps != 0:
                continue

            plt.plot()
            plt.scatter(points_parents, points_children)
            plt.title("Correlation " + str(correlation))
            x = np.nanmean(points_parents)
            y = np.nanmean(points_children)
            min_x = np.min(points_parents)
            max_x = np.max(points_parents)
            points = np.linspace(min_x, max_x)
            m = correlation
            b = y - (x * m)
            plt.plot(points, b + m * points, '-', color="red")

            min_y = np.min(points_parents)
            max_y = np.max(points_parents)
            min_a = min(min_x, min_y)
            max_a = max(max_x, max_y)

            plt.xlim(left=min_a, right=max_a)
            plt.ylim(bottom=min_a, top=max_a)
            plt.xlabel("generations")
            plt.ylabel("corrcoef average parents vs child")
            plt.show()

        generations = [i for i in range(0, len(self.generational_history)-1, 1)]
        plt.plot(generations, correlations)
        plt.title("Generational heritability")
        plt.show()


class IndividualMetrics(Metrics):

    def __init__(self):
        super().__init__()
        self.measures: Dict[Measurement, List] = dict()

        self.initialize()

    def initialize(self):
        for measurement_type in [BehavioralMeasurement, EpigeneticMeasurement, MorphologicalMeasurement]:
            for measurement in measurement_type:
                self.measures[measurement] = []

    def log(self, individuals):
        for individual in individuals:
            for measure_key in individual.measures.keys():
                self.measures[measure_key].append(individual.measures[measure_key])


class PopulationMetrics(Dict[str, List]):

    def __init__(self):
        super().__init__()
        self["rejection_rate"] = []
        self["acceptance_rate"] = []
        self.old_individual_ids = []

    def log(self, individuals, rejected_individuals):
        total_individuals: int = len(individuals)
        total_offspring: int = len(rejected_individuals)

        count_rejection: int = 0
        count_acceptance: int = 0

        for individual in individuals:
            if individual.id not in self.old_individual_ids:
                count_acceptance += 1
                self.old_individual_ids.append(individual.id)

        for individual in rejected_individuals:
            if individual.id in self.old_individual_ids:
                self.old_individual_ids.remove(individual.id)
                count_rejection += 1

        self["acceptance_rate"].append(count_acceptance / total_individuals)
        if total_offspring > 0:
            self["rejection_rate"].append(count_rejection / total_offspring)
        else:
            self["rejection_rate"].append(0.0)

    def __repr__(self):
        output = ""
        output += "rejection: " + str(self["rejection_rate"]) + "\n"
        output += "acceptance: " + str(self["acceptance_rate"]) + "\n"
        return output
