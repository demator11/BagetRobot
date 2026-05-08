from typing import Protocol
from ...models.pose import Pose
from ...models.command import RobotCommand


class NavigationStrategy(Protocol):
    """Протокол стратегии навигации - как добраться до цели."""

    def compute_command(
            self,
            current_pose: Pose,
            target_x: float,
            target_y: float
    ) -> RobotCommand:
        """
        По текущей позиции и цели выдать команду управления.
        Для дифференциального привода: сначала поворот, потом движение.
        """
        ...

    def is_target_reached(
            self,
            current_pose: Pose,
            target_x: float,
            target_y: float,
            tolerance: float = 0.1
    ) -> bool:
        """Достигли ли цели?"""
        ...