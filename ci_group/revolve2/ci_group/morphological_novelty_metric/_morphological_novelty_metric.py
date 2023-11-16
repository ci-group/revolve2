from dataclasses import dataclass
from math import atan2, pi, sqrt

import numpy as np
from numpy.typing import NDArray

from revolve2.modular_robot import ModularRobot

from ._coordinate_operations import coords_from_bodies
from .calculate_novelty import calculate_novelty


@dataclass(init=False)
class MorphologicalNoveltyMetric:
    """
    Calculate the Morphological Novelty Score for a Population.

    This metric for Morphological Novelty considers robots a distribution in space, which can be reshaped into any other distribution.
    The work that has to be done to reshape distribution 1 to distribution 2 is used for the final novelty calculation.

    A detailed description of the Algorithm can be found in:
    Oliver Weissl, and A.E. Eiben. "Morphological-Novelty in Modular Robot Evolution". 2023 IEEE Symposium Series on Computational Intelligence (SSCI)(pp. 1066-1071). IEEE, 2023.
    """

    _coordinates: list[NDArray[np.float128]]
    _magnitudes: list[list[float]]
    _orientations: list[list[tuple[float, float]]]
    _histograms: NDArray[np.float128]
    _int_histograms: NDArray[np.int64]
    _novelty_scores: NDArray[np.float64]

    _NUM_BINS: int = 20
    """The amount of bins in the histogram. Increasing this allows for more detail, but risks sparseness, while lower values generalize more."""
    _INT_CASTER: int = 10_000
    """Casting floats to INT allows to mitigate floating-point issues in the distribution reshaping. The higher the number, the more presicion you get."""

    def get_novelty_from_population(
        self,
        population: list[ModularRobot],
        cob_heuristic: bool = False,
    ) -> list[float]:
        """
        Get the morphological novelty score for individuals in a population.

        :param population: The population of robots.
        :param cob_heuristic: Whether the heuristic approximation for change of basis is used.
        :return: The novelty scores.
        """
        instances = len(population)
        bodies = [robot.body for robot in population]

        self._coordinates = coords_from_bodies(bodies, cob_heuristics=cob_heuristic)

        self._histograms = np.empty(
            shape=(instances, self._NUM_BINS, self._NUM_BINS), dtype=np.float64
        )
        self._int_histograms = np.empty(
            shape=(instances, self._NUM_BINS, self._NUM_BINS), dtype=np.int64
        )

        self._coordinates_to_magnitudes_orientation()
        self._gen_gradient_histogram()
        self._normalize_cast_int()

        self._novelty_scores = calculate_novelty(
            self._int_histograms, self._int_histograms.shape[0], self._NUM_BINS
        )
        max_novelty = self._novelty_scores.max()
        return [float(score / max_novelty) for score in self._novelty_scores]

    def _coordinates_to_magnitudes_orientation(self) -> None:
        """Calculate the magnitude and orientation for the coordinates supplied."""
        instances = len(self._coordinates)
        self._magnitudes = [[0.0]] * instances
        self._orientations = [[(0.0, 0.0)]] * instances
        for i in range(instances):
            coordinates_amount = self._coordinates[i].shape[0]
            magnitudes = [0.0] * coordinates_amount
            orientations = [(0.0, 0.0)] * coordinates_amount
            for j in range(coordinates_amount):
                coord = self._coordinates[i][j]
                ax = atan2(sqrt(coord[1] ** 2 + coord[2] ** 2), coord[0]) * 180 / pi
                az = atan2(coord[2], sqrt(coord[1] ** 2 + coord[0] ** 2)) * 180 / pi
                orientations[j] = (ax, az)
                magnitudes[j] = sqrt(coord.dot(coord))
            self._orientations[i] = orientations
            self._magnitudes[i] = magnitudes

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
                self._histograms[i, x, z] += magnitude

    def _normalize_cast_int(self) -> None:
        """Normalize a matrix (array), making its sum  = _INT_CASTER."""
        instances = self._histograms.shape[0]
        for i in range(instances):
            histogram = self._histograms[i].copy()
            histogram = np.array(
                (histogram / histogram.sum()) * self._INT_CASTER, dtype=np.int64
            )  # Casting the float histograms to int, in order to avoid floating point errors in the reshaping.

            error = (
                self._INT_CASTER - histogram.sum()
            )  # Due to the int-casting the histogram sums are marginally smaller than _INT_CASTER.
            mask = np.zeros(shape=histogram.size, dtype=np.int64)
            mask[
                :error
            ] += 1  # each histogram must sum up to _INT_CASTER. Therefore, a mask is applied.
            np.random.seed(42)  # forces shuffles into reproducible patterns
            np.random.shuffle(mask)  # shuffling the mask to avoid bias in the histogram
            self._int_histograms[i] = histogram + np.reshape(
                mask, (-1, histogram.shape[0])
            )
