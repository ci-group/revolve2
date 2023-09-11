from dataclasses import dataclass

from revolve2.core.physics.running import geometry


@dataclass
class Terrain:
    """Terrain describing part of a physics `Environment`."""

    static_geometry: list[geometry.Geometry]
