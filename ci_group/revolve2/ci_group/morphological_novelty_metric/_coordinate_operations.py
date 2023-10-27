from dataclasses import dataclass, field
from itertools import product
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from revolve2.modular_robot.body import Module
from revolve2.modular_robot.body.base import Body


@dataclass
class CoordinateOperations:
    """Transform points in a distribution."""

    _coords: list[NDArray[np.float64]] = field(init=False)

    def coords_from_bodies(
        self, bodies: list[Body], cob_heuristics: bool = True
    ) -> list[NDArray[np.float64]]:
        """
        Extract coordinates of modules from a body.

        :param bodies: The bodies.
        :param cob_heuristics: If change of basis heuristic approximation is used.
        :return: The array of coordinates.
        """
        self._body_to_adjusted_coordinates(bodies)
        if cob_heuristics:
            self._coordinates_pca_heuristic()
        else:
            self._coordinates_pca_change_basis()
        return self._coords

    def _body_to_adjusted_coordinates(self, bodies: list[Body]) -> None:
        """
        Extract coordinates of modules in a body and adjusts them with the core position.

        :param bodies: The body.
        """
        self._coords = [np.empty(shape=0)] * len(bodies)
        i = 0
        for body in bodies:
            body_array, core_position = body.to_grid()
            body_np_array: NDArray[Any] = np.asarray(body_array)

            x, y, z = body_np_array.shape

            elements = []
            for xe, ye, ze in product(range(x), range(y), range(z)):
                target = body_np_array[xe][ye][ze]
                if isinstance(target, Module):
                    elements.append(np.subtract((xe, ye, ze), core_position))
            self._coords[i] = np.asarray(elements)
            i += 1

    def _coordinates_pca_change_basis(self) -> None:
        """
        Transform the coordinate distribution by the magnitude of variance of the respective basis.

        The detailed steps of the transformation are discussed in the paper.
        """
        i = 0
        for coordinates in self._coords:
            if len(coordinates) > 1:
                covariance_matrix = np.cov(coordinates.T)
                eigen_values, eigen_vectors = np.linalg.eig(covariance_matrix)

                srt = np.argsort(eigen_values)[
                    ::-1
                ]  # sorting axis by amplitude of variance
                for i in range(len(srt)):
                    while True:
                        if srt[i] == i:
                            break
                        candidate = srt[i]
                        rotation = Rotation.from_rotvec(
                            np.radians(180) * eigen_vectors[candidate]
                        )
                        coordinates = rotation.apply(coordinates)

                        eigen_vectors[i], eigen_vectors[candidate] = np.copy(
                            eigen_vectors[candidate]
                        ), np.copy(eigen_vectors[i])
                        srt[[i, candidate]] = srt[[candidate, i]]

                coordinates = np.linalg.inv(eigen_vectors).dot(coordinates.T)
                self._coords[i] = coordinates.T
            i += 1

    def _coordinates_pca_heuristic(self) -> None:
        """
        Transform the coordinate distribution by the magnitude of variance of the respective basis.

        The heuristic approximation of the transformation by simply switching axes.
        """
        i = 0
        for coordinates in self._coords:
            if len(coordinates) > 1:
                covariance_matrix = np.cov(coordinates.T)
                eigen_values, _ = np.linalg.eig(covariance_matrix)
                srt = np.argsort(eigen_values)[::-1]
                for j in range(len(srt)):
                    while True:
                        if srt[j] == j:
                            break
                        candidate = srt[j]
                        coordinates[:, [j, candidate]] = coordinates[:, [candidate, j]]
                        srt[[j, candidate]] = srt[[candidate, j]]
                self._coords[i] = coordinates
            i += 1
