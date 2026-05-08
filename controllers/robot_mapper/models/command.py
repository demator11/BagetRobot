from dataclasses import dataclass


@dataclass
class RobotCommand:
    """Команды управления дифференциальным приводом."""
    linear_velocity: float  # м/с (вперёд/назад)
    angular_velocity: float  # рад/с (+ = поворот против часовой)

    def stop(self) -> "RobotCommand":
        return RobotCommand(linear_velocity=0.0, angular_velocity=0.0)

    def rotate(self, omega: float) -> "RobotCommand":
        return RobotCommand(linear_velocity=0.0, angular_velocity=omega)

    def move_forward(self, speed: float = 0.5) -> "RobotCommand":
        return RobotCommand(linear_velocity=speed, angular_velocity=0.0)