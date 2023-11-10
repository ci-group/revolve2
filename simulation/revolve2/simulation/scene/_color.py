from dataclasses import dataclass


@dataclass
class Color:
    """
    Represents a color in RGBA format.

    All values should from 0 to 255.
    """

    red: int
    green: int
    blue: int
    alpha: int

    def to_normalized_rgba_list(self) -> list[float]:
        """
        Convert to rgba list where each value is between 0 and 1.

        :returns: The list.
        """
        return [self.red / 255, self.green / 255, self.blue / 255, self.alpha / 255]

    def to_normalized_rgb_list(self) -> list[float]:
        """
        Convert to rgb list where each value is between 0 and 1.

        :returns: The list.
        """
        return [self.red / 255, self.green / 255, self.blue / 255]
