import os
from dataclasses import dataclass, field
from math import atan2, pi, sqrt

import numpy as np
from numpy.typing import NDArray

from revolve2.modular_robot import ModularRobot

from ._coordinate_operations import CoordinateOperations


@dataclass
class MorphologicalNoveltyMetric:
    """Calculate the Morphological Novelty Score for a Population."""

    _coordinates: list[NDArray[np.float64]] = field(init=False)
    _magnitudes: list[list[float]] = field(init=False)
    _orientations: list[list[tuple[float, float]]] = field(init=False)
    _histograms: NDArray[np.float64] = field(init=False)
    _int_histograms: NDArray[np.int64] = field(init=False)
    _novelty_scores: NDArray[np.float64] = field(init=False)

    _NUM_BINS: int = 20  # amount of bins for the histogram descriptor
    _INT_CASTER: int = 10_000  # to counteract floating-point issues

    def get_novelty_from_population(
        self, population: list[ModularRobot]
    ) -> list[float]:
        """
        Get the morphological novelty score for individuals in a population.

        :param population: The population of robots.
        :return: The novelty scores.
        :raises ModuleNotFoundError: If the cython module for novelty calculation is not present.
        """
        instances = len(population)
        bodies = [robot.body for robot in population]

        self._coordinates = CoordinateOperations().coords_from_bodies(bodies)
        self._histograms = np.empty(
            shape=(instances, self._NUM_BINS, self._NUM_BINS), dtype=np.float64
        )
        self._int_histograms = np.empty(
            shape=(instances, self._NUM_BINS, self._NUM_BINS), dtype=np.int64
        )
        self._coordinates_to_magnitudes_orientation()
        self._gen_gradient_histogram()
        self._wasserstein_softmax()

        self._check_cmodule()
        try:
            # noinspection PyUnresolvedReferences
            from .calculate_novelty import calculate_novelty  # type: ignore
        except ModuleNotFoundError:
            raise ModuleNotFoundError(
                "The calculate_novelty module was not built properly."
            )

        self._novelty_scores = calculate_novelty(self._int_histograms)
        max_novelty = self._novelty_scores.max()
        novelty_scores = [float(score / max_novelty) for score in self._novelty_scores]
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
        """Generate the gradient histograms for the respective histogram index."""
        bin_size = int(360 / self._NUM_BINS)
        assert (
            bin_size == 360 / self._NUM_BINS
        ), "Error: num_bins has to be a divisor of 360"
        instances = len(self._coordinates)

        for i in range(instances):
            for orientation, magnitude in zip(
                self._orientations[i], self._magnitudes[i]
            ):
                x, z = (int(orientation[0] / bin_size), int(orientation[1] / bin_size))
                self._histograms[i][x][z] += magnitude * self._INT_CASTER

    @staticmethod
    def _check_cmodule() -> None:
        """Check weather cmodule is built."""
        directory_path = os.path.dirname(os.path.abspath(__file__))
        if not any(
            [
                file.startswith("calculate_novelty")
                for file in os.listdir(directory_path)
            ]
        ):
            os.chdir(directory_path)
            os.system("python -m _build_cmodule build_ext -i")


    def _wasserstein_softmax(self) -> None:
        """Calculate a softmax for an array, making it sum = _INT_CASTER"""
        instances = self._histograms.shape[0]
        for i in range(instances):
            array = self._histograms[i].copy()
            array += 1 / array.size
            array = np.array(np.true_divide(array, array.sum()) * self._INT_CASTER, dtype=np.int64)

            error = self._INT_CASTER - np.sum(array)
            mask = np.zeros(shape=array.shape)
            mask[:error] = 1
            np.random.shuffle(mask)

            self._int_histograms[i] = array + mask
