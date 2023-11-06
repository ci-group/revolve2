from dataclasses import dataclass, field
from math import atan2, pi, sqrt

import numpy as np
from numpy.typing import NDArray

from revolve2.modular_robot import ModularRobot

from ._coordinate_operations import CoordinateOperations
from .calculate_novelty import calculate_novelty  # type: ignore


@dataclass
class MorphologicalNoveltyMetric:
    """Calculate the Morphological Novelty Score for a Population."""

    _coordinates: list[NDArray[np.float128]] = field(init=False)
    _magnitudes: list[list[float]] = field(default_factory=lambda: [[0.0]])
    _orientations: list[list[tuple[float, float]]] = field(
        default_factory=lambda: [[(0.0, 0.0)]]
    )
    _histograms: NDArray[np.float128] = field(
        default_factory=lambda: np.empty(1, dtype=np.float128)
    )
    _int_histograms: NDArray[np.int64] = field(
        default_factory=lambda: np.empty(1, dtype=np.int64)
    )
    _novelty_scores: NDArray[np.float64] = field(init=False)

    _NUM_BINS: int = 20  # amount of bins for the histogram descriptor
    _INT_CASTER: int = 10_000  # to counteract floating-point issues

    def get_novelty_from_population(
        self,
        population: list[ModularRobot],
        cob_heuristic: bool = False,
    ) -> list[float]:
        """
        Get the morphological novelty score for individuals in a population.

        :param population: The population of robots.
        :param cob_heuristic: Weather the heuristic approximation of change of basis is used.
        :return: The novelty scores.
        """
        instances = len(population)
        bodies = [robot.body for robot in population]

        self._coordinates = CoordinateOperations().coords_from_bodies(
            bodies, cob_heuristics=cob_heuristic
        )

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
        novelty_scores = [float(score / max_novelty) for score in self._novelty_scores]
        return novelty_scores

    def _coordinates_to_magnitudes_orientation(self) -> None:
        """Calculate the magnitude and orientation for the coordinates supplied."""
        instances = len(self._coordinates)
        self._magnitudes *= instances
        self._orientations *= instances
        for i in range(instances):
            coordinates_amount = len(self._coordinates[i])
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
                self._histograms[i][x][z] += magnitude

    def _normalize_cast_int(self) -> None:
        """Normalize a matrix (array), making its sum  = _INT_CASTER."""
        instances = self._histograms.shape[0]
        for i in range(instances):
            histogram = self._histograms[i].copy()
            histogram = np.array(
                (histogram / histogram.sum()) * self._INT_CASTER, dtype=np.int64
            )

            error = self._INT_CASTER - histogram.sum()
            mask = np.zeros(shape=histogram.size, dtype=np.int64)
            mask[:error] += 1
            self._int_histograms[i] = histogram + np.reshape(
                mask, (-1, histogram.shape[0])
            )
