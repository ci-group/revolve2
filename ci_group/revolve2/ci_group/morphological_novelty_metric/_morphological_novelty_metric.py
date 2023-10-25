import os
from os.path import join, isfile
from revolve2.modular_robot import ModularRobot
from numpy.typing import NDArray
from math import atan2, pi, sqrt
from dataclasses import dataclass, field
import numpy as np
from ._coordinate_operations import CoordinateOperations

@dataclass
class MorphologicalNoveltyMetric:
    """Calculate the Morphological Novelty Score for a Population."""
    _coordinates: list[NDArray[float]] = field(init=False)
    _magnitudes: list[list[float]] = field(init=False)
    _orientations: list[list[tuple[float, float]]] = field(init=False)
    _histograms: NDArray[float] = field(init=False)

    _NUM_BINS: int = 20  # amount of bins for the histogram descriptor

    def get_novelty_from_population(self, population: list[ModularRobot]) -> list:
        """
        Get the morphological novelty score for individuals in a population.

        :param population: The population of robots.
        :return: The novelty scores.
        :raises ModuleNotFoundError: If the cython module for novelty calculation is not present.
        """
        instances = len(population)
        bodies = [robot.body for robot in population]

        self._coordinates = CoordinateOperations.coords_from_bodies(bodies)
        self._histograms = np.empty(shape=(instances, self._NUM_BINS, self._NUM_BINS), dtype=float)
        self._coordinates_to_magnitudes_orientation()
        self._gen_gradient_histogram()

        directory_path = os.path.dirname(os.path.abspath(__file__))

        if not any([file.startswith("calculate_novelty") for file in os.listdir(directory_path)]):
            os.chdir(directory_path)
            os.system("python -m _build_cmodule build_ext -i")
        try:
            # noinspection PyUnresolvedReferences
            from .calculate_novelty import calculate_novelty
        except ModuleNotFoundError:
            raise ModuleNotFoundError("The calculate_novelty module was not built properly.")

        novelty_scores = calculate_novelty(self._histograms)
        max_novelty = max(novelty_scores)
        novelty_scores = [float(score / max_novelty) for score in novelty_scores]
        return novelty_scores

    def _coordinates_to_magnitudes_orientation(self) -> None:
        """Calculate the magnitude and orientation for the coordinates supplied."""
        instances = len(self._coordinates)
        self._magnitudes = [[0.0] * instances for _ in range(instances)]
        self._orientations = [[(0.0, 0.0)] * instances for _ in range(instances)]
        for i in range(instances):
            j = 0
            for coord in self._coordinates[i]:
                if len(coord) == 3:
                    ax = atan2(sqrt(coord[1] ** 2 + coord[2] ** 2), coord[0]) * 180 / pi
                    az = atan2(coord[2], sqrt(coord[1] ** 2 + coord[0] ** 2)) * 180 / pi
                    self._orientations[i][j] = (ax, az)
                    self._magnitudes[i][j] = sqrt(coord.dot(coord))
                j += 1

    def _gen_gradient_histogram(self) -> None:
        """
        Generate the gradient histograms for the respecive histogram index.

        :param histogram_index: The target index.
        """
        bin_size = int(360 / self._NUM_BINS)
        assert bin_size == 360 / self._NUM_BINS, "Error: num_bins has to be a divisor of 360"
        instances = len(self._coordinates)

        for i in range(instances):
            for orientation, magnitude in zip(self._orientations[i], self._magnitudes[i]):
                x, z = (int(orientation[0] / bin_size), int(orientation[1] / bin_size))
                self._histograms[i][x][z] += magnitude
            self._histograms[i] = self._wasserstein_softmax(self._histograms[i])

    @staticmethod
    def _wasserstein_softmax(array: NDArray[float]) -> NDArray[float]:
        """
        Calculate a softmax for an array, making it sum = 1.

        :param array: The array.
        :return: The normalized array.
        """
        array += (1 / array.size)
        return np.true_divide(array, array.sum())
