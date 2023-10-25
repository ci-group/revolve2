import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
from itertools import product
from revolve2.modular_robot.body.base import Body
from revolve2.modular_robot.body import Module


class CoordinateOperations:

    @classmethod
    def coords_from_bodies(cls, bodies: list[Body], cob_heuristics: bool = True) -> list[NDArray[float]]:
        """
        Extract coordinates of modules from a body.

        :param bodies: The bodies.
        :param cob_heuristics: If change of basis heuristic approximation is used.
        :return: The array of coordinates.
        """
        coords = [cls._body_to_adjusted_coordinates(body) for body in bodies]
        if cob_heuristics:
            return [cls._coordinates_pca_heuristic(coord) for coord in coords]
        return [cls._coordinates_pca_change_basis(coord) for coord in coords]

    @classmethod
    def _body_to_adjusted_coordinates(cls, body: Body) -> NDArray[float]:
        """
        Extract coordinates of modules in a body and adjusts them with the core position.

        :param body: The body.
        :return: The coordinates of the Modules.
        """
        body_array, core_position = body.to_grid()
        body_array = np.asarray(body_array)

        x, y, z = body_array.shape

        elements = []
        for xe, ye, ze in product(range(x), range(y), range(z)):
            target = body_array[xe][ye][ze]
            if isinstance(target, Module):
                elements.append(np.subtract((xe, ye, ze), core_position))
        return np.asarray(elements)

    @classmethod
    def _coordinates_pca_change_basis(cls, coordinates: NDArray[float]) -> NDArray[float]:
        """
        Transform the coordinate distribution by the magnitude of variance of the respective basis.
        The detailed steps of the transformation are discussed in the paper.

        :param coordinates: The list of points in space.
        :return: The transformed list of points.
        """
        if len(coordinates) > 1:
            covariance_matrix = np.cov(coordinates.T)
            eigen_values, eigen_vectors = np.linalg.eig(covariance_matrix)

            srt = np.argsort(eigen_values)[::-1]  # sorting axis by amplitude of variance
            for i in range(len(srt)):
                while True:
                    if srt[i] == i:
                        break
                    candidate = srt[i]
                    rotation = Rotation.from_rotvec(np.radians(180) * eigen_vectors[candidate])
                    coordinates = rotation.apply(coordinates)

                    eigen_vectors[i], eigen_vectors[candidate] = np.copy(eigen_vectors[candidate]), np.copy(eigen_vectors[i])
                    srt[[i, candidate]] = srt[[candidate, i]]

            coordinates = np.linalg.inv(eigen_vectors).dot(coordinates.T)
        return coordinates.T

    @classmethod
    def _coordinates_pca_heuristic(cls, coordinates: NDArray[float]) -> NDArray[float]:
        """
        Transform the coordinate distribution by the magnitude of variance of the respective basis.
        The heuristic approximation of the transformation by simply switching axes.

        :param coordinates: The list of points in space.
        :return: The transformed list of points.
        """

        if len(coordinates) > 1:
            covariance_matrix = np.cov(coordinates.T)
            eigen_values, _ = np.linalg.eig(covariance_matrix)
            srt = np.argsort(eigen_values)[::-1]
            for i in range(len(srt)):
                while True:
                    if srt[i] == i:
                        break
                    candidate = srt[i]
                    coordinates[:, [i, candidate]] = coordinates[:, [candidate, i]]
                    srt[[i, candidate]] = srt[[candidate, i]]
        return coordinates
