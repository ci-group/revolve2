from dataclasses import dataclass

from ._geometry import Geometry


@dataclass(kw_only=True)
class GeometrySphere(Geometry):
    """Box geometry."""

    radius: float
    """The radius of the sphere."""
