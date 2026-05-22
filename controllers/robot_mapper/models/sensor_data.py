from dataclasses import dataclass
from typing import List, Optional


@dataclass
class SensorData:
    """Данные с датчиков робота на одном шаге симуляции."""
    distances: List[float]  # массив расстояний до препятствий, inf если нет препятствия
    angles: List[float]  # углы обзора каждого датчика (относительно робота)

    def min_distance(self) -> float:
        return min(self.distances)

    def min_front_distance(self, front_angle_threshold: float = 0.6) -> float:
        front_dists = [
            d for d, a in zip(self.distances, self.angles)
            if abs(a) <= front_angle_threshold
        ]
        return min(front_dists) if front_dists else float("inf")

    def get_obstacle_direction(self) -> Optional[float]:
        if not self.distances:
            return None
        min_idx = self.distances.index(min(self.distances))
        return self.angles[min_idx]

    def left_distance(self) -> float:
        left_dists = [d for d, a in zip(self.distances, self.angles) if a < 0]
        return min(left_dists) if left_dists else float("inf")

    def right_distance(self) -> float:
        right_dists = [d for d, a in zip(self.distances, self.angles) if a > 0]
        return min(right_dists) if right_dists else float("inf")
