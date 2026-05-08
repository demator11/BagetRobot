from typing import Tuple, Set
from .pose import Pose


class CoverageMap:
    """
    Карта покрытия (где побывал робот).

    Клетка считается покрытой ТОЛЬКО когда робот доехал до неё.
    - Цитата призрачного Миши
    """

    def __init__(self, resolution: float = 0.1, map_size: int = 500):
        self.resolution = resolution
        self.map_size = map_size
        self.visited: Set[Tuple[int, int]] = set()

    def _world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Конвертировать мировые координаты в индекс клетки."""
        cx = int(x / self.resolution + self.map_size / 2)
        cy = int(y / self.resolution + self.map_size / 2)
        return cx, cy

    def mark_visited(self, pose: Pose) -> None:
        """
        Отметить клетку как покрытую (робот доехал).
        """
        cx, cy = self._world_to_cell(pose.x, pose.y)
        self.visited.add((cx, cy))

        # Добавляем соседние клетки (если робот занимает место)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                self.visited.add((cx + dx, cy + dy))

    def coverage_percent(self) -> float:
        """Процент покрытия от всего пространства."""
        total_cells = self.map_size * self.map_size
        return (len(self.visited) / total_cells) * 100.0

    def is_visited(self, x: float, y: float) -> bool:
        """Проверка, была ли посещена клетка."""
        cell = self._world_to_cell(x, y)
        return cell in self.visited