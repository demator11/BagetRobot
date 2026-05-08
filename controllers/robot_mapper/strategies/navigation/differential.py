import math
from ...models.pose import Pose
from ...models.command import RobotCommand
from .base import NavigationStrategy


class DifferentialDriveNavigation(NavigationStrategy):
    """
    Навигация для дифференциального привода.
    Этапы: 1) повернуться к цели, 2) ехать прямо.
    """

    def __init__(
            self,
            max_linear_speed: float = 0.5,  # м/с
            max_angular_speed: float = 2.0,  # рад/с
            angle_tolerance: float = 0.05,  # рад (~3 градуса)
            position_tolerance: float = 0.1,  # метры
    ):
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.angle_tolerance = angle_tolerance
        self.position_tolerance = position_tolerance
        self._is_aligned = False

    def compute_command(
            self,
            current_pose: Pose,
            target_x: float,
            target_y: float
    ) -> RobotCommand:
        # 1. Вычисляем целевой угол
        target_angle = math.atan2(target_y - current_pose.y, target_x - current_pose.x)

        # 2. Вычисляем разницу углов (нормализуем)
        angle_diff = target_angle - current_pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # 3. Если не повернулись к цели - вращаемся
        if abs(angle_diff) > self.angle_tolerance:
            self._is_aligned = False
            angular_speed = self.max_angular_speed * (1 if angle_diff > 0 else -1)
            return RobotCommand(linear_velocity=0.0, angular_velocity=angular_speed)

        # 4. Повернулись - едем вперёд
        self._is_aligned = True
        distance = math.hypot(target_x - current_pose.x, target_y - current_pose.y)

        # Если почти доехали - тормозим
        if distance < self.position_tolerance:
            return RobotCommand.stop()

        return RobotCommand(linear_velocity=self.max_linear_speed, angular_velocity=0.0)

    def is_target_reached(
            self,
            current_pose: Pose,
            target_x: float,
            target_y: float,
            tolerance: float = 0.1
    ) -> bool:
        distance = math.hypot(target_x - current_pose.x, target_y - current_pose.y)
        return distance < tolerance