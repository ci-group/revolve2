from dataclasses import dataclass

from revolve2.simulation.scene.geometry import Geometry


@dataclass
class Terrain:
    """Terrain consising of only static geometry."""

    static_geometry: list[Geometry]
    """The static geometry that defines the terrain."""
