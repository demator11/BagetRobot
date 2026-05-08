import math
from typing import Optional, Tuple

from ...models.pose import Pose
from ...models.coverage_map import CoverageMap
from .base import ExplorationStrategy


class GreedyExploration(ExplorationStrategy):
    """
    Средний уровень: едет к ближайшей непокрытой клетке вокруг.
    """

    def __init__(
            self,
            search_radius: float = 3.0,
            min_target_distance: float = 0.5,
    ):
        self.search_radius = search_radius
        self.min_target_distance = min_target_distance
        self.complete = False
        self.last_target: Optional[Tuple[float, float]] = None

    def get_next_target(self, current_pose, coverage_map):
        best_target = None
        best_distance = float('inf')

        uncovered_count = 0

        step = 0.2
        radius = self.search_radius

        for dx in range(int(-radius / step), int(radius / step) + 1):
            for dy in range(int(-radius / step), int(radius / step) + 1):
                x = current_pose.x + dx * step
                y = current_pose.y + dy * step
                dist = math.hypot(x - current_pose.x, y - current_pose.y)

                if dist < self.min_target_distance:
                    continue

                if not coverage_map.is_visited(x, y):
                    uncovered_count += 1
                    if dist < best_distance:
                        best_distance = dist
                        best_target = (x, y)

        if best_target is None:
            return self._expand_search(current_pose, coverage_map)

        return best_target

    def _expand_search(
            self,
            current_pose: Pose,
            coverage_map: CoverageMap,
    ) -> Optional[Tuple[float, float]]:
        """Поиск на больших радиусах."""
        for radius in [3.0, 5.0, 8.0, 10.0]:
            step = 0.5  # грубый поиск на больших дистанциях
            for dx in range(int(-radius / step), int(radius / step) + 1):
                for dy in range(int(-radius / step), int(radius / step) + 1):
                    x = current_pose.x + dx * step
                    y = current_pose.y + dy * step
                    dist = math.hypot(x - current_pose.x, y - current_pose.y)

                    if dist < self.min_target_distance:
                        continue

                    if not coverage_map.is_visited(x, y):
                        return x, y
        return None

    def is_complete(self) -> bool:
        return self.complete

    def reset(self) -> None:
        self.complete = False
        self.last_target = None
