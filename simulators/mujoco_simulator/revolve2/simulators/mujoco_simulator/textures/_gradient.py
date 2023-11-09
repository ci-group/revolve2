from dataclasses import dataclass, field

from revolve2.simulation.scene import Color
from revolve2.simulation.scene.geometry.textures import Texture, TextureReference

from ._mujoco_builtin_references import MujocoBuiltinReferences


@dataclass(kw_only=True, frozen=True)
class Gradient(Texture):
    """A color gradient spanning over geometric models."""

    reference: TextureReference = field(default=MujocoBuiltinReferences.GRADIENT.value)

    primary_color: Color
    secondary_color: Color
