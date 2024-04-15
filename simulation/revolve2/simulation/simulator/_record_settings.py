from dataclasses import dataclass
from typing import Optional


@dataclass
class RecordSettings:
    """Settings for recording a simulation."""

    video_directory: str
    overwrite: bool = False

    fps: int = 24

    width: Optional[int] = None
    height: Optional[int] = None

    # offscreen: bool = False << For future implementation (student-proof?)
