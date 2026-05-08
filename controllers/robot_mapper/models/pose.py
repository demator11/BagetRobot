from dataclasses import dataclass
import math


@dataclass
class Pose:
    """Положение робота в 2D пространстве."""
    x: float
    y: float
    theta: float  # радианы, 0 = смотрит на восток

    def distance_to(self, other: "Pose") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def to_tuple(self) -> tuple[float, float]:
        return self.x, self.y
