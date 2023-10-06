from dataclasses import dataclass


@dataclass
class RecordSettings:
    """Settings for recording a simulation."""

    video_directory: str
    fps: int = 24
