from abc import ABC
from dataclasses import dataclass

from ..._color import Color


@dataclass
class Texture(ABC):
    """An abstract texture for mujoco models."""

    _name: str
    _primary_color: Color
    _secondary_color: Color
    _repeat: tuple[int, int]
    _size: tuple[int, int]
    _map_type: str
    """
    map_type represents how the texture is mapped to an object.
    
    - "2d": Maps the texture to a 2d surface.
    - "cube": Wraps the texture around a cube object.
    - "skybox": Like "cube" but maps onto the inside of an object. 
    """

    _specular: float  # ranges [0, 1]
    _shininess: float  # ranges [0, 1]
    _reflectance: float  # ranges [0, 1]
    _emission: float

    @property
    def name(self) -> str:
        """
        Get the name for mujoco.

        :return: The name.
        """
        return self._name

    @property
    def primary_color(self) -> Color:
        """
        Get the primary color.

        :return: The color.
        """
        return self._primary_color

    @property
    def secondary_color(self) -> Color:
        """
        Get the secondary color.

        :return: The color.
        """
        return self._secondary_color

    @property
    def map_type(self) -> str:
        """
        Get the map type.

        :return: The type.
        """
        return self._map_type

    @property
    def repeat(self) -> tuple[int, int]:
        """
        Get how often a texture should be repeated across x and y dimensions.

        :return: The amount.
        """
        return self._repeat

    @property
    def size(self) -> tuple[int, int]:
        """
        Get the x and y size of the texture.

        :return: The x and y size.
        """
        return self._size

    @property
    def specular(self) -> float:
        """
        Get how specular a texture should be.

        :return: The value.
        """
        return self._specular

    @property
    def shininess(self) -> float:
        """
        Get how shiny a surface should be.

        :return: The value.
        """
        return self._shininess

    @property
    def reflectance(self) -> float:
        """
        Get how reflective a surface should be.

        :return: The value.
        """
        return self._reflectance

    @property
    def emission(self) -> float:
        """
        Get the emissiveness of a surface.

        :return: The value.
        """
        return self._emission
