from dataclasses import dataclass


@dataclass
class RobotCommand:
    """Differential drive command."""
    linear_velocity: float  # m/s
    angular_velocity: float  # rad/s

    @classmethod
    def stop(cls) -> "RobotCommand":
        return cls(linear_velocity=0.0, angular_velocity=0.0)

    @classmethod
    def rotate(cls, omega: float) -> "RobotCommand":
        return cls(linear_velocity=0.0, angular_velocity=omega)

    @classmethod
    def move_forward(cls, speed: float = 0.5) -> "RobotCommand":
        return cls(linear_velocity=speed, angular_velocity=0.0)
