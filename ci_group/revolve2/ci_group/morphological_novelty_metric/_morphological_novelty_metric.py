from math import atan2, pi, sqrt

import numpy as np
from numpy.typing import NDArray
from revolve2.modular_robot import ModularRobot

from ._coordinate_operations import coords_from_bodies
from .calculate_novelty import calculate_novelty

_INT_CASTER: int = 10_000
"""Casting floats to INT allows to mitigate floating-point issues in the distribution reshaping. The higher the number, the more precision you get."""

Orientations = list[list[tuple[float, float]]]
Magnitudes = list[list[float]]


def get_novelty_from_population(
    population: list[ModularRobot], cob_heuristic: bool = False, num_bins: int = 20
) -> NDArray[np.float64]:
    """
    Get the morphological novelty score for individuals in a population.

    This metric for Morphological Novelty considers robots a distribution in space, which can be reshaped into any other distribution.
    The work that has to be done to reshape distribution 1 to distribution 2 is used for the final novelty calculation.
    A detailed description of the Algorithm can be found in:
    Oliver Weissl, and A.E. Eiben. "Morphological-Novelty in Modular Robot Evolution". 2023 IEEE Symposium Series on Computational Intelligence (SSCI)(pp. 1066-1071). IEEE, 2023.

    :param population: The population of robots.
    :param cob_heuristic: Whether the heuristic approximation for change of basis is used.
    :param num_bins: The amount of bins in the histogram. Increasing this allows for more detail, but risks sparseness, while lower values generalize more.
    :return: The novelty scores.
    """
    bodies = [robot.body for robot in population]

    coordinates = coords_from_bodies(bodies, cob_heuristics=cob_heuristic)

    magn, orient = _coordinates_to_magnitudes_orientation(coordinates)
    histograms = _gen_gradient_histogram(
        orientations=orient, magnitudes=magn, num_bins=num_bins
    )
    int_histograms = _normalize_cast_int(histograms)

    novelty_scores: NDArray[np.float64] = calculate_novelty(
        int_histograms, int_histograms.shape[0], num_bins
    )
    return novelty_scores


def _coordinates_to_magnitudes_orientation(
    coordinates: list[NDArray[np.float64]],
) -> tuple[Magnitudes, Orientations]:
    """
    Calculate the magnitude and orientation for the coordinates supplied.

    :param coordinates: The coordinates for calculating the magnitudes and orientations.
    :return: The magnitudes and orientations.
    """
    instances = len(coordinates)
    magnitudes = [[0.0]] * instances
    orientations = [[(0.0, 0.0)]] * instances
    for i in range(instances):
        coordinates_amount = coordinates[i].shape[0]
        mags = [0.0] * coordinates_amount
        orts = [(0.0, 0.0)] * coordinates_amount
        for j in range(coordinates_amount):
            coord = coordinates[i][j]
            ax = atan2(sqrt(coord[1] ** 2 + coord[2] ** 2), coord[0]) * 180 / pi
            az = atan2(coord[2], sqrt(coord[1] ** 2 + coord[0] ** 2)) * 180 / pi
            orts[j] = (ax, az)
            mags[j] = sqrt(coord.dot(coord))
        orientations[i] = orts
        magnitudes[i] = mags
    return magnitudes, orientations


def _gen_gradient_histogram(
    orientations: Orientations,
    magnitudes: Magnitudes,
    num_bins: int,
) -> NDArray[np.float64]:
    """
    Generate the gradient histograms for the respective histogram index.

    :param orientations: The orientations of points in the bodies.
    :param magnitudes: The magnitudes of points in the bodies.
    :param num_bins: The number of bins in the histogram.
    :return: The gradient histograms. Shape = (instances x num_bins x num_bins).
    """
    bin_size = 360 / num_bins
    instances = len(orientations)
    histograms = np.zeros(shape=(instances, num_bins, num_bins), dtype=np.float64)
    for i in range(instances):
        for orientation, magnitude in zip(orientations[i], magnitudes[i]):
            x, z = int(orientation[0] / bin_size), int(orientation[1] / bin_size)
            histograms[i, x, z] += magnitude
    return histograms


def _normalize_cast_int(histograms: NDArray[np.float64]) -> NDArray[np.int64]:
    """
    Normalize a matrix (array), making its sum = _INT_CASTER.

    :param histograms: The histograms to cast and normalize.
    :return: The normalized and cast histograms.
    """
    int_histograms = np.zeros(histograms.shape, dtype=np.int64)

    instances = histograms.shape[0]
    for i in range(instances):
        histogram = histograms[i].copy()

        if histogram.sum() > 0.0:
            histogram /= histogram.sum()
        else:
            # If a body only has a core, but no other modules the histogram will be empty.
            histogram += 1 / histogram.size

        histogram *= _INT_CASTER
        # Casting the float histograms to int, in order to avoid floating point errors in the reshaping.
        histogram = histogram.astype(np.int64)

        # Due to the int-casting the histogram sums are marginally smaller than _INT_CASTER.
        error = _INT_CASTER - histogram.sum()
        mask = np.zeros(shape=histogram.size, dtype=np.int64)

        # each histogram must sum up to _INT_CASTER. Therefore, a mask is applied.
        mask[:error] += 1
        np.random.seed(42)  # forces shuffles into reproducible patterns
        np.random.shuffle(mask)  # shuffling the mask to minimize bias in the histogram
        int_histograms[i] = histogram + np.reshape(mask, (-1, histogram.shape[0]))

    return int_histograms
