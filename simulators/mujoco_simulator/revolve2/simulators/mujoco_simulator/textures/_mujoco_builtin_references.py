from enum import Enum
from revolve2.simulation.scene.geometry.textures import TextureReference


class MujocoBuiltinReferences(Enum):
    FLAT = TextureReference(builtin="flat")
    GRADIENT = TextureReference(builtin="gradient")
    CHECKER = TextureReference(builtin="checker")
