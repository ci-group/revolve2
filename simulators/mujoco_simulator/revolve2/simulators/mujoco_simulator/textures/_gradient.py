from dataclasses import dataclass, field

from revolve2.simulation.scene import Color
from revolve2.simulation.scene.geometry.textures import Texture, TextureReference


@dataclass(kw_only=True, frozen=True)
class Gradient(Texture):
    """A color gradient spanning over geometric models."""

    reference: TextureReference = field(
        default_factory=lambda: TextureReference(builtin="gradient")
    )

    primary_color: Color
    secondary_color: Color
