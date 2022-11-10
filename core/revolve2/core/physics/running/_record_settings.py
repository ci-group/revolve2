from dataclasses import dataclass


@dataclass
class RecordSettings:
    """Settings for recording physics running."""

    video_directory: str
    fps: int = 24
