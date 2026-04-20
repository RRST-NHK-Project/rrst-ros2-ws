from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass
class KfsDetectionResult:
    detected: bool
    map_x: float = 0.0
    map_y: float = 0.0
    yaw_deg: float = 0.0
    scale: float = 1.0
    match_count: int = 0
    inlier_ratio: float = 0.0
    depth_mm: float = float("nan")


class DepthFusionAdapter(Protocol):
    """Adapter interface for depth integration that can evolve independently."""

    def update_depth_observation(self, data: list[float]) -> None: ...

    def inject_depth(self, detection: KfsDetectionResult) -> KfsDetectionResult: ...


class NoOpDepthFusionAdapter:
    """Default adapter that keeps detection independent from depth sources."""

    def __init__(self) -> None:
        self._latest_depth_mm = float("nan")

    def update_depth_observation(self, data: list[float]) -> None:
        if len(data) > 0:
            self._latest_depth_mm = float(data[0])

    def inject_depth(self, detection: KfsDetectionResult) -> KfsDetectionResult:
        if detection.detected:
            detection.depth_mm = self._latest_depth_mm
        return detection
