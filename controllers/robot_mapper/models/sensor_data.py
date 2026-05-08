from dataclasses import dataclass
from typing import List, Optional


@dataclass
class SensorData:
    """Данные с датчиков робота на одном шаге симуляции."""
    distances: List[float]  # массив расстояний до препятствий, inf если нет препятствия
    angles: List[float]  # углы обзора каждого датчика (относительно робота)

    def min_distance(self) -> float:
        return min(self.distances)

    def get_obstacle_direction(self) -> Optional[float]:
        """Вернуть направление ближайшего препятствия или None."""
        min_idx = self.distances.index(min(self.distances))
        return self.angles[min_idx]