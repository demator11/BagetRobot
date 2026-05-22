from typing import Optional, Protocol

from ...models.command import RobotCommand
from ...models.pose import Pose
from ...models.sensor_data import SensorData


class NavigationStrategy(Protocol):

    def compute_command(
            self,
            current_pose: Pose,
            target_x: float,
            target_y: float,
            sensor_data: Optional[SensorData] = None,
    ) -> RobotCommand:
        ...

    def is_target_reached(
            self,
            current_pose: Pose,
            target_x: float,
            target_y: float,
            tolerance: float | None = None
    ) -> bool:
        ...
