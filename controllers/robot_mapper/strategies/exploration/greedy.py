import math
from typing import Optional, Tuple

from ...models.pose import Pose
from ...models.coverage_map import CoverageMap
from .base import ExplorationStrategy


class GreedyExploration(ExplorationStrategy):
    """
    Средний уровень: едет к ближайшей непокрытой клетке вокруг.
    """

    def __init__(self, search_radius: float = 1.5):
        self.search_radius = search_radius
        self.complete = False
        self.last_target: Optional[Tuple[float, float]] = None

    def get_next_target(
            self,
            current_pose: Pose,
            coverage_map: CoverageMap,
    ) -> Optional[Tuple[float, float]]:
        # Сканируем вокруг в поиске непокрытых точек
        best_target = None
        best_distance = float('inf')

        # Проверяем точки на сетке в радиусе search_radius
        step = 0.1  # метров
        radius = self.search_radius

        for dx in range(int(-radius / step), int(radius / step) + 1):
            for dy in range(int(-radius / step), int(radius / step) + 1):
                x = current_pose.x + dx * step
                y = current_pose.y + dy * step

                if not coverage_map.is_visited(x, y):
                    dist = math.hypot(x - current_pose.x, y - current_pose.y)
                    if dist < best_distance:
                        best_distance = dist
                        best_target = (x, y)

        if best_target is None:
            # Если всё вокруг посещено — расширяем радиус
            return self._expand_search(current_pose, coverage_map)

        self.last_target = best_target
        return best_target

    def _expand_search(
            self,
            current_pose: Pose,
            coverage_map: CoverageMap,
    ) -> Optional[Tuple[float, float]]:
        """Расширенный поиск, если рядом нет непокрытых клеток."""
        for radius in [2.0, 3.0, 5.0]:
            for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
                rad = math.radians(angle)
                x = current_pose.x + radius * math.cos(rad)
                y = current_pose.y + radius * math.sin(rad)

                if not coverage_map.is_visited(x, y):
                    return x, y
        return None

    def is_complete(self) -> bool:
        return self.complete

    def reset(self) -> None:
        self.complete = False
        self.last_target = None