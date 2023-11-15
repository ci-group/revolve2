"""
Calculate Morphological Novelty across a Population.

Using the MorphologicalNoveltyMetric for evaluating a population of robots can be done as shown in the code below. Note that once the object is instantiated, you can evaluate different populations without making a new instance.

 >>> robots: list[revolve2.modular_robot.ModularRobot]
 >>> mnm = MorphologicalNoveltyMetric()
 >>> novelties = mnm.get_novelty_from_population(robots)
"""

from ._morphological_novelty_metric import MorphologicalNoveltyMetric

__all__ = ["MorphologicalNoveltyMetric"]
