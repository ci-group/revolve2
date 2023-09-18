from revolve2.simulation.actor._color import Color


class Properties:
    """A class to store and expand properties of modular robots easily."""

    attachment_position: int
    color: Color
    num_children: int
    rotation: float

    def __init__(
        self,
        rotation: float,
        num_children: int = 0,
        attachment_position: int = 5,
        color: Color = Color(0, 0, 0, 1),
    ):
        """
        Initialize this object.

        :param rotation: The modules rotation.
        :param num_children: The max number of children.
        :param attachment_position: The attachment position of a module.
        :param color: The modules color.
        """
        self.num_children = num_children
        self.rotation = rotation
        self.attachment_position = attachment_position
        self.color = color
