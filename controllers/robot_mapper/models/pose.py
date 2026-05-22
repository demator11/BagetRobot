import math
from dataclasses import dataclass


@dataclass
class Pose:
    x: float
    y: float
    theta: float  # радианы, 0 = восток, + = против часовой

    def distance_to(self, other: "Pose") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def angle_to(self, other: "Pose") -> float:
        return math.atan2(other.y - self.y, other.x - self.x)
