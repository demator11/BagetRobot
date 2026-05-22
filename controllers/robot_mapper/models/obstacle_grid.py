import math
from typing import Optional, Tuple

import numpy as np

FREE = 0.0
WALL = 1.0
UNKNOWN = 0.5


class OccupancyGrid:
    """Карта занятости с препятствиями.

    - 0.0 = свободно
    - 1.0 = стена
    - 0.5 = неизвестно (не используется)
    """

    def __init__(self, map_size: int = 200, resolution: float = 0.2):
        self.map_size = map_size
        self.resolution = resolution
        self.grid = np.full((map_size, map_size), FREE)

    def _world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        cx = int(x / self.resolution + self.map_size / 2)
        cy = int(y / self.resolution + self.map_size / 2)
        return cx, cy

    def _cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        x = (cx - self.map_size / 2) * self.resolution
        y = (cy - self.map_size / 2) * self.resolution
        return x, y

    def is_free(self, x: float, y: float) -> bool:
        cx, cy = self._world_to_cell(x, y)
        if not (0 <= cx < self.map_size and 0 <= cy < self.map_size):
            return False
        return self.grid[cy, cx] != WALL

    def path_is_free(
        self, x1: float, y1: float, x2: float, y2: float
    ) -> bool:
        dx = x2 - x1
        dy = y2 - y1
        dist = math.hypot(dx, dy)
        steps = max(int(dist / self.resolution * 4), 8)
        for i in range(1, steps):
            t = i / steps
            if not self.is_free_with_clearance(x1 + t * dx, y1 + t * dy, 0.15):
                return False
        return True

    def add_rectangle(
        self, cx: float, cy: float, width: float, height: float
    ):
        """Закрасить прямоугольную область как стену."""
        half_w = width / 2
        half_h = height / 2
        x_min, _ = self._world_to_cell(cx - half_w, cy)
        x_max, _ = self._world_to_cell(cx + half_w, cy)
        _, y_min = self._world_to_cell(cx, cy - half_h)
        _, y_max = self._world_to_cell(cx, cy + half_h)

        x_min = max(0, min(x_min, x_max))
        x_max = max(0, max(x_min, x_max), min(x_max, self.map_size - 1))
        y_min = max(0, min(y_min, y_max))
        y_max = max(0, min(y_max, self.map_size - 1))

        self.grid[y_min : y_max + 1, x_min : x_max + 1] = WALL

    def is_free_with_clearance(
        self, x: float, y: float, clearance: float = 0.3
    ) -> bool:
        """Проверить, что точка и область вокруг неё свободны."""
        if not self.is_free(x, y):
            return False
        step = self.resolution
        for dx in range(-int(clearance / step), int(clearance / step) + 1):
            for dy in range(-int(clearance / step), int(clearance / step) + 1):
                if not self.is_free(x + dx * step, y + dy * step):
                    return False
        return True

    def find_detour(
        self, x1: float, y1: float, x2: float, y2: float
    ) -> Optional[Tuple[float, float]]:
        """Найти свободную точку в обход препятствия на пути.

        Ищет сбоку от первой стены на луче к цели.
        Возвращает точку с буфером 0.3 м от стен.
        """
        dx = x2 - x1
        dy = y2 - y1
        dist = math.hypot(dx, dy)
        if dist < 0.01:
            return None

        nx = dx / dist
        ny = dy / dist
        px = -ny
        py = nx

        steps = max(int(dist / self.resolution * 3), 5)
        for i in range(1, steps):
            t = i / steps
            bx = x1 + t * dx
            by = y1 + t * dy
            if self.is_free(bx, by):
                continue

            for side in (1, -1):
                for offset_m in (0.5, 0.8, 1.2, 1.6, 2.0):
                    wx = bx + px * side * offset_m
                    wy = by + py * side * offset_m
                    if self.is_free_with_clearance(wx, wy):
                        return (wx, wy)

            for radius in (0.5, 0.8, 1.2, 1.6, 2.0):
                for angle_deg in range(0, 360, 30):
                    rad = math.radians(angle_deg)
                    wx = bx + math.cos(rad) * radius
                    wy = by + math.sin(rad) * radius
                    if self.is_free_with_clearance(wx, wy):
                        return (wx, wy)

        return None

    def to_numpy(self) -> np.ndarray:
        return self.grid
