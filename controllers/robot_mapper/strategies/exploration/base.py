from typing import Protocol, Optional, Tuple
import numpy as np

from ...models.pose import Pose


class ExplorationStrategy(Protocol):
    """Протокол стратегии исследования."""

    def get_next_target(
            self,
            current_pose: Pose,
            occupancy_grid: np.ndarray,
    ) -> Optional[Tuple[float, float]]:
        """
        Определить следующую цель для движения.

        Args:
            current_pose: текущее положение робота
            occupancy_grid: 2D массив, где 0 = свободно, 1 = стена, 0.5 = неизвестно

        Returns:
            Координаты (x, y) следующей цели или None если исследование завершено
        """
        ...

    def is_complete(self) -> bool:
        """Вернуть True если исследование закончено."""
        ...

    def reset(self) -> None:
        """Сбросить состояние (для нового запуска)."""
        ...