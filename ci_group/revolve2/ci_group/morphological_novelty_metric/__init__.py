"""
Calculate Morphological Novelty across a Population.

Using the MorphologicalNoveltyMetric for evaluating a population of robots can be done as follows:

 >>> robots: list[revolve2.modular_robot.ModularRobot]
 >>> mnm = MorphologicalNoveltyMetric()
 >>> novelties = mnm.get_novelty_from_population(robots)
"""

from ._morphological_novelty_metric import MorphologicalNoveltyMetric

__all__ = ["MorphologicalNoveltyMetric"]
