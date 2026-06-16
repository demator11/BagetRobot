import math
from typing import Optional, Tuple

from ...models.command import RobotCommand
from ...models.pose import Pose
from ...models.sensor_data import SensorData
from .base import NavigationStrategy


class DifferentialDriveNavigation(NavigationStrategy):

    def __init__(
        self,
        max_linear_speed: float = 0.5,
        max_angular_speed: float = 2.0,
        angle_tolerance: float = 0.08,
        position_tolerance: float = 0.15,
        obstacle_grid=None,
        safe_distance: float = 0.35,
    ):
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.angle_tolerance = angle_tolerance
        self.position_tolerance = position_tolerance
        self.safe_distance = safe_distance
        self.obstacle_grid = obstacle_grid

        self._state = "DRIVE"
        self._target_theta = 0.0
        self._path: list = []
        self._path_index = 0
        self._blocked_steps = 0
        self._drive_angle_tol = angle_tolerance * 2.0

    def _pick_exit_dir(self, sd: SensorData) -> float:
        left_dist = sd.left_distance()
        right_dist = sd.right_distance()
        if left_dist >= right_dist:
            return math.pi / 2
        return -math.pi / 2

    def _count_unknown_ahead(self, cell_x: int, cell_y: int, dx: int, dy: int, max_cells: int = 15) -> int:
        if self.obstacle_grid is None:
            return 0
        grid = self.obstacle_grid.grid
        ms = self.obstacle_grid.map_size
        count = 0
        for step in range(1, max_cells + 1):
            nx = cell_x + dx * step
            ny = cell_y + dy * step
            if 0 <= nx < ms and 0 <= ny < ms:
                if grid[ny, nx] == 0.5:
                    count += 1
                elif grid[ny, nx] == 1.0:
                    break
            else:
                break
        return count

    def _choose_turn_angle(self, sd: SensorData, pose: Pose) -> float:
        if self.obstacle_grid is None:
            return self._pick_exit_dir(sd)
        cx, cy = self.obstacle_grid._world_to_cell(pose.x, pose.y)
        head_x = int(round(math.cos(pose.theta)))
        head_y = int(round(math.sin(pose.theta)))
        left_x = -head_y
        left_y = head_x
        right_x = head_y
        right_y = -head_x
        left_score = self._count_unknown_ahead(cx, cy, left_x, left_y)
        right_score = self._count_unknown_ahead(cx, cy, right_x, right_y)
        back_score = self._count_unknown_ahead(cx, cy, -head_x, -head_y)
        if left_score >= right_score and left_score >= back_score:
            return math.pi / 2
        if right_score >= back_score:
            return -math.pi / 2
        return math.pi

    def compute_command(
        self,
        current_pose: Pose,
        target_x: float,
        target_y: float,
        sensor_data: Optional[SensorData] = None,
    ) -> RobotCommand:
        obstacle = False
        if sensor_data is not None:
            front = sensor_data.min_front_distance(0.6)
            side = sensor_data.min_distance()
            on_path = self._path and self._path_index < len(self._path)
            if on_path:
                if side < 0.13:
                    obstacle = True
            else:
                if front < self.safe_distance or side < 0.25:
                    obstacle = True

        if self._state == "TURN":
            err = self._target_theta - current_pose.theta
            err = math.atan2(math.sin(err), math.cos(err))
            if abs(err) < self.angle_tolerance:
                self._state = "DRIVE"
                self._blocked_steps = 0
                return RobotCommand.move_forward(self.max_linear_speed)
            ang = self.max_angular_speed * (1 if err > 0 else -1)
            if sensor_data is not None and sensor_data.min_distance() < 0.15:
                return RobotCommand(linear_velocity=-0.2, angular_velocity=ang)
            return RobotCommand(linear_velocity=0.0, angular_velocity=ang)

        if obstacle:
            self._blocked_steps += 1
            if self._blocked_steps >= 5:
                self._path = []
                self._path_index = 0
                angle = self._choose_turn_angle(sensor_data, current_pose)
                self._target_theta = current_pose.theta + angle
                self._target_theta = math.atan2(
                    math.sin(self._target_theta), math.cos(self._target_theta)
                )
                self._state = "TURN"
                if sensor_data is not None and sensor_data.min_distance() < 0.15:
                    return RobotCommand(linear_velocity=-0.2, angular_velocity=0.0)
                return RobotCommand.stop()
            return RobotCommand(linear_velocity=-0.1, angular_velocity=0.0)

        self._blocked_steps = 0
        if not self._path:
            tx, ty = target_x, target_y
            dist = math.hypot(tx - current_pose.x, ty - current_pose.y)
            if dist < self.position_tolerance:
                return RobotCommand.stop()
            if self.obstacle_grid is not None:
                p = self.obstacle_grid.find_path(
                    current_pose.x, current_pose.y, tx, ty
                )
                if p and len(p) > 1:
                    self._path = p
                    self._path_index = 0

        if self._path and self._path_index < len(self._path):
            wx, wy = self._path[self._path_index]
            dx = wx - current_pose.x
            dy = wy - current_pose.y
            if math.hypot(dx, dy) < self.position_tolerance:
                self._path_index += 1
                if self._path_index >= len(self._path):
                    self._path = []
                    self._path_index = 0
                    return RobotCommand.move_forward(self.max_linear_speed)
                wx, wy = self._path[self._path_index]

            target_angle = math.atan2(wy - current_pose.y, wx - current_pose.x)
            err = target_angle - current_pose.theta
            err = math.atan2(math.sin(err), math.cos(err))
            if abs(err) > self._drive_angle_tol:
                self._target_theta = target_angle
                self._state = "TURN"
                ang = self.max_angular_speed * (1 if err > 0 else -1)
                return RobotCommand(linear_velocity=0.0, angular_velocity=ang)
            return RobotCommand.move_forward(self.max_linear_speed)

        target_angle = math.atan2(target_y - current_pose.y, target_x - current_pose.x)
        err = target_angle - current_pose.theta
        err = math.atan2(math.sin(err), math.cos(err))
        if abs(err) > self._drive_angle_tol:
            self._target_theta = target_angle
            self._state = "TURN"
            ang = self.max_angular_speed * (1 if err > 0 else -1)
            return RobotCommand(linear_velocity=0.0, angular_velocity=ang)
        return RobotCommand.move_forward(self.max_linear_speed)

    def is_target_reached(
        self,
        current_pose: Pose,
        target_x: float,
        target_y: float,
        tolerance: float | None = None,
    ) -> bool:
        tol = self.position_tolerance if tolerance is None else tolerance
        return math.hypot(target_x - current_pose.x, target_y - current_pose.y) < tol