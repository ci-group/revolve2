from dataclasses import dataclass, field


from ._mujoco_builtin_references import MujocoBuiltinReferences
from revolve2.simulation.scene import Color
from revolve2.simulation.scene.geometry.textures import Texture, TextureReference


@dataclass(kw_only=True, frozen=True)
class Checker(Texture):
    """A checker texture for geometric models."""

    reference: TextureReference = field(default=MujocoBuiltinReferences.CHECKER.value)
    repeat: tuple[int, int] = field(default=(100, 100))

    primary_color: Color
    secondary_color: Color
