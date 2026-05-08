import math
import random
from typing import Optional, Tuple

from ...models.coverage_map import CoverageMap
from ...models.pose import Pose
from .base import ExplorationStrategy


class RandomExploration(ExplorationStrategy):
    """
    Минимальный уровень: случайное движение.
    Выбирает случайную точку, потом едет к ней.
    """

    def __init__(
            self,
            random_range: float = 1.0,
            max_steps: int = 100,
    ):
        self.random_range = random_range
        self.max_steps = max_steps
        self.steps_taken = 0
        self.current_target: Optional[Tuple[float, float]] = None

    def get_next_target(
            self,
            current_pose: Pose,
            coverage_map: CoverageMap,
    ) -> Optional[Tuple[float, float]]:

        if self.current_target is not None:
            return self.current_target

        self.steps_taken += 1

        if self.steps_taken > self.max_steps:
            return None

        # Случайная точка вокруг робота
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(0.3, self.random_range)

        target_x = current_pose.x + distance * math.cos(angle)
        target_y = current_pose.y + distance * math.sin(angle)

        self.current_target = (target_x, target_y)
        return self.current_target

    def reset_target(self) -> None:
        """Сбросить текущую цель (вызывается извне при достижении)."""
        self.current_target = None

    def is_complete(self) -> bool:
        return self.steps_taken > self.max_steps

    def reset(self) -> None:
        self.steps_taken = 0
        self.current_target = None