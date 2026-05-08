from typing import List, Tuple

import numpy as np


def ray_cast(
        start: Tuple[float, float],
        angle: float,
        max_range: float,
        resolution: float = 0.05,
) -> List[Tuple[float, float]]:
    """
    Сгенерировать точки вдоль луча для проверки препятствий.

    Returns:
        Список точек (x, y) от start до max_range
    """
    points = []
    steps = int(max_range / resolution)
    for i in range(1, steps + 1):
        dist = i * resolution
        x = start[0] + dist * np.cos(angle)
        y = start[1] + dist * np.sin(angle)
        points.append((x, y))
    return points


def world_to_grid(
        x: float,
        y: float,
        map_origin: Tuple[float, float] = (0, 0),
        resolution: float = 0.05
) -> Tuple[int, int]:
    """Конвертировать мировые координаты в индексы карты."""
    gx = int((x - map_origin[0]) / resolution)
    gy = int((y - map_origin[1]) / resolution)
    return gx, gy