from ..._color import Color
from ._map_type import MapType
from ._texture import Texture


class Gradient(Texture):
    """A color gradient spanning over mujoco models."""

    def __init__(
        self,
        color1: Color,
        color2: Color,
        map_type: MapType = MapType.CUBE,
        size: tuple[int, int] = (10, 10),
        repeat: tuple[int, int] = (1, 1),
        specular: float = 0.5,
        shininess: float = 0.5,
        reflectance: float = 0.0,
        emission: float = 0.0,
    ) -> None:
        """
        Initialize a gradient of two colors for some geometry.

        :param color1: The first color.
        :param color2: The second color.
        :param map_type: How the texture should be mapped.
        :param size: The size of the texture.
        :param repeat: How often the texture should be repeated.
        :param specular: How specular a texture is.
        :param shininess: How shiny a texture is.
        :param reflectance: How reflective a texture is.
        :param emission: How emissive a surface is.
        """
        self.name = "gradient"
        self.primary_color = color1
        self.secondary_color = color2
        self.map_type = map_type
        self.repeat = repeat
        self.size = size
        self.specular = specular
        self.shininess = shininess
        self.reflectance = reflectance
        self.emission = emission
