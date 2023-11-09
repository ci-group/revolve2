from dataclasses import dataclass, field

from revolve2.simulation.scene import Color
from revolve2.simulation.scene.geometry.textures import Texture, TextureReference


@dataclass(kw_only=True, frozen=True)
class Flat(Texture):
    """A flat texture for geometric models."""

    reference: TextureReference = field(
        default_factory=lambda: TextureReference(builtin="flat")
    )
    primary_color: Color
