import math
from typing import Optional, Tuple

from ...models.command import RobotCommand
from ...models.pose import Pose
from .base import NavigationStrategy


class DifferentialDriveNavigation(NavigationStrategy):
    """
    Навигация для дифференциального привода.
    Этапы: 1) повернуться к цели, 2) ехать прямо.
    Если на пути препятствие — строит waypoint в обход.
    """

    def __init__(
        self,
        max_linear_speed: float = 0.5,
        max_angular_speed: float = 2.0,
        angle_tolerance: float = 0.05,
        position_tolerance: float = 0.1,
        obstacle_grid: Optional = None,
    ):
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.angle_tolerance = angle_tolerance
        self.position_tolerance = position_tolerance
        self._is_aligned = False
        self.obstacle_grid = obstacle_grid
        self._waypoint: Optional[Tuple[float, float]] = None

    def compute_command(
        self,
        current_pose: Pose,
        target_x: float,
        target_y: float,
    ) -> RobotCommand:
        tx, ty = self._resolve_target(current_pose, target_x, target_y)

        target_angle = math.atan2(ty - current_pose.y, tx - current_pose.x)
        angle_diff = target_angle - current_pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        if abs(angle_diff) > self.angle_tolerance:
            self._is_aligned = False
            angular_speed = self.max_angular_speed * (
                1 if angle_diff > 0 else -1
            )
            return RobotCommand(linear_velocity=0.0, angular_velocity=angular_speed)

        self._is_aligned = True
        distance = math.hypot(tx - current_pose.x, ty - current_pose.y)

        if distance < self.position_tolerance:
            return RobotCommand.stop()

        return RobotCommand(
            linear_velocity=self.max_linear_speed, angular_velocity=0.0
        )

    def _resolve_target(
        self, pose: Pose, target_x: float, target_y: float
    ) -> Tuple[float, float]:
        """Вернуть цель напрямую или waypoint в обход препятствия."""
        if self.obstacle_grid is None:
            return target_x, target_y

        if self._waypoint is not None:
            wx, wy = self._waypoint
            reached = (
                math.hypot(wx - pose.x, wy - pose.y) < self.position_tolerance
            )
            if not reached:
                return wx, wy

        if self.obstacle_grid.path_is_free(
            pose.x, pose.y, target_x, target_y
        ):
            self._waypoint = None
            return target_x, target_y

        detour = self.obstacle_grid.find_detour(
            pose.x, pose.y, target_x, target_y
        )
        if detour is not None:
            self._waypoint = detour
            return detour

        return target_x, target_y

    def is_target_reached(
        self,
        current_pose: Pose,
        target_x: float,
        target_y: float,
        tolerance: float = 0.1,
    ) -> bool:
        if self._waypoint is not None:
            return False

        distance = math.hypot(
            target_x - current_pose.x, target_y - current_pose.y
        )
        return distance < tolerance
