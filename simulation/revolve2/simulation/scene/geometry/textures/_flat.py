from attrs import field
from attrs.setters import frozen

from ..._color import Color
from ._map_type import MapType
from ._texture import Texture


class Flat(Texture):
    """A flat texture for mujoco models."""

    translucent: bool = field(on_setattr=frozen)

    def __init__(
        self,
        color1: Color,
        map_type: MapType = MapType.CUBE,
        size: tuple[int, int] = (10, 10),
        repeat: tuple[int, int] = (1, 1),
        specular: float = 0.5,
        shininess: float = 0.5,
        reflectance: float = 0.0,
        emission: float = 0.0,
        translucent: bool = False,
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
        :param translucent: If the surface should be translucent.
        """
        self.name = "flat"
        self.primary_color = self.secondary_color = color1
        self.map_type = map_type
        self.repeat = repeat
        self.size = size
        self.specular = specular
        self.shininess = shininess
        self.reflectance = reflectance
        self.emission = emission
        self.translucent = translucent
