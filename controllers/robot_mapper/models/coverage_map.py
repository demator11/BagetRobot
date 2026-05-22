from typing import Dict, Tuple

import numpy as np

from .pose import Pose


class CoverageMap:
    """Карта покрытия (где побывал робот и сколько раз)."""

    def __init__(self, resolution: float = 0.1, map_size: int = 500):
        self.resolution = resolution
        self.map_size = map_size
        self.visits: Dict[Tuple[int, int], int] = {}

    def _world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        cx = int(x / self.resolution + self.map_size / 2)
        cy = int(y / self.resolution + self.map_size / 2)
        return cx, cy

    def mark_visited(self, pose: Pose) -> None:
        cx, cy = self._world_to_cell(pose.x, pose.y)
        key = (cx, cy)
        self.visits[key] = self.visits.get(key, 0) + 1

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nk = (cx + dx, cy + dy)
                self.visits[nk] = self.visits.get(nk, 0) + 1

    def coverage_percent(self) -> float:
        total_cells = self.map_size * self.map_size
        return (len(self.visits) / total_cells) * 100.0

    def is_visited(self, x: float, y: float) -> bool:
        cell = self._world_to_cell(x, y)
        return cell in self.visits

    def get_visit_grid(self, map_size: int = 0) -> np.ndarray:
        """Вернуть 2D массив счётчиков посещений.

        0 = не посещено, 1 = один раз, >=2 = многократно.
        """
        if map_size == 0:
            map_size = self.map_size
        grid = np.zeros((map_size, map_size), dtype=np.int32)
        for (cx, cy), count in self.visits.items():
            if 0 <= cx < map_size and 0 <= cy < map_size:
                grid[cy, cx] = count
        return grid
