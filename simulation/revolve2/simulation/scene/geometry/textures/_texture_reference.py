from abc import ABC
from dataclasses import dataclass, field


@dataclass(frozen=True, kw_only=True)
class TextureReference(ABC):
    """Reference texture information for the simulators."""

    builtin: str | None = field(default=None)
    """Allows to reference builtin textures of the targeted simulator."""
    file: str | None = field(default=None)
    """The source file of the specified texture, if custom."""
    content_type: str | None = field(default=None)
    """The type of the source file."""
    gridlayout: str | None = field(default=None)
    """Specify the layout of the texture to be mapped onto an object."""

    def __post_init__(self) -> None:
        """
        Check for potential conflicts due to missing arguments.

        :raises NotImplementedError: If not sufficient arguments are given.
        """
        if self.builtin is None and self.file is None:
            raise NotImplementedError(
                "No texture reference given in form of a file or builtin texture."
            )
        if self.file is not None and self.content_type is None:
            raise NotImplementedError(
                "Please indicate the content type of your texture source file."
            )

    def get_kwargs(self) -> dict[str, str]:
        """
        Get all non-None kwargs for the simulator.

        :return: The kwargs in form of a dict.
        """
        return {k: v for k, v in self.__dict__.items() if v is not None}
