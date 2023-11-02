from ..._color import Color
from ._texture import Texture


class Flat(Texture):
    """A flat texture for mujoco models."""

    def __init__(
        self,
        color1: Color,
        map_type: str = "cube",
        size: tuple[int, int] = (10, 10),
        repeat: tuple[int, int] = (1, 1),
        specular: float = 0.5,
        shininess: float = 0.5,
        reflectance: float = 0.0,
        emission: float = 0.0,
    ) -> None:
        """
        Initialize a flat color for some geometry.

        :param color1: The color for the object.
        :param map_type: How the texture should be mapped.
        :param size: The size of the texture.
        :param repeat: How often the texture should be repeated.
        :param specular: How specular a texture is.
        :param shininess: How shiny a texture is.
        :param reflectance: How reflective a texture is.
        :param emission: How emissive a surface is.
        """
        self._name = "flat"
        self._primary_color = self._secondary_color = color1
        self._map_type = map_type
        self._repeat = repeat
        self._size = size
        self._specular = specular
        self._shininess = shininess
        self._reflectance = reflectance
        self._emission = emission
