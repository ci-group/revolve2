from dataclasses import dataclass

from .running import geometry


@dataclass
class Terrain:
    """Terrain describing part of a physics `Environment`."""

    static_geometry: list[geometry.Geometry]
