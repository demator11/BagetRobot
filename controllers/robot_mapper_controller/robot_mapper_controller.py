import math
import random
import sys
import time
from dataclasses import dataclass
from pathlib import Path

_CONTROLLER_DIR = Path(__file__).resolve().parent
_PROJECT_DIR = _CONTROLLER_DIR.parent
if str(_PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(_PROJECT_DIR))

from robot_mapper.models.coverage_map import CoverageMap
from robot_mapper.models.obstacle_grid import OccupancyGrid
from robot_mapper.models.pose import Pose
from robot_mapper.strategies.exploration.frontier import FrontierExploration
from robot_mapper.strategies.navigation.differential import (
    DifferentialDriveNavigation,
)
from webots_robot import WebotsRobot


@dataclass
class Config:

    map_size: int = 200
    map_resolution: float = 0.1
    coverage_resolution: float = 0.2

    search_radius: float = 4.0
    min_target_dist: float = 0.8

    max_linear_speed: float = 0.5   # m/s
    max_angular_speed: float = 1.5  # rad/s
    angle_tolerance: float = 0.08   # rad (~4.6Р В Р’В Р вЂ™Р’В Р В Р вЂ Р В РІР‚С™Р Р†РІР‚С›РЎС›Р В Р’В Р Р†Р вЂљРІвЂћСћР В РІР‚в„ўР вЂ™Р’В°)
    position_tolerance: float = 0.15  # m
    safe_distance: float = 0.3
    slow_distance: float = 0.6

    log_interval: int = 50

def main():
    cfg = Config()

    print("[Controller] Initialising WebotsRobot...")
    robot = WebotsRobot()
    timestep_s = robot.timestep / 1000.0
    print(f"  Timestep: {robot.timestep} ms ({timestep_s} s)")

    obstacle_grid = OccupancyGrid(
        map_size=cfg.map_size,
        resolution=cfg.map_resolution,
    )
    coverage = CoverageMap(
        resolution=cfg.coverage_resolution,
        map_size=int(cfg.map_size * cfg.map_resolution / cfg.coverage_resolution),
    )

    exploration = FrontierExploration(
        occupancy_grid=obstacle_grid,
        min_frontier_size=5,
        search_margin=30,
    )
    navigation = DifferentialDriveNavigation(
        max_linear_speed=cfg.max_linear_speed,
        max_angular_speed=cfg.max_angular_speed,
        angle_tolerance=cfg.angle_tolerance,
        position_tolerance=cfg.position_tolerance,
        obstacle_grid=obstacle_grid,
        safe_distance=cfg.safe_distance,
    )

    current_target: tuple[float, float] | None = None
    current_target_steps = 0
    escape_steps = 0
    step_count = 0
    targets_reached = 0
    last_log_step = 0
    last_pose = Pose(0.0, 0.0, 0.0)
    idle_steps = 0
    _checkpoint = Pose(0.0, 0.0, 0.0)
    _checkpoint_steps = 0
    last_obstacle_log = -999

    print("[Controller] Entering main loop...")
    start_wall = time.time()

    while robot.step() != -1:
        step_count += 1
        pose = robot.read_pose()
        sonar_data = robot.read_sonars()

        min_d = sonar_data.min_distance()
        if min_d < 1.0 and step_count - last_obstacle_log > 20:
            print(
                f"  [SONAR] step={step_count} min={min_d:.2f}m  "
                f"left={sonar_data.left_distance():.2f}m  "
                f"right={sonar_data.right_distance():.2f}m  "
                f"pose=({pose.x:.2f},{pose.y:.2f})  "
                f"distances={[f'{d:.1f}' if d < 10 else 'inf' for d in sonar_data.distances]}"
            )
            last_obstacle_log = step_count
        area_before = coverage.coverage_percent()
        obstacle_grid.update_from_sonar(
            pose=pose,
            distances=sonar_data.distances,
            angles=sonar_data.angles,
            max_range=3.0,
        )
        coverage.mark_visited(pose)
        _checkpoint_steps += 1
        if _checkpoint_steps > 30:
            if pose.distance_to(_checkpoint) < 0.3:
                idle_steps += 1
            else:
                _checkpoint = Pose(pose.x, pose.y, pose.theta)
                _checkpoint_steps = 0
                idle_steps = 0
        last_pose = pose
        if current_target is None:
            current_target = exploration.get_next_target(pose, coverage)
            current_target_steps = 0
        else:
            current_target_steps += 1
            if current_target_steps > 200:
                print(f"  [Controller] Target timeout at step {step_count} Р В Р’В Р вЂ™Р’В Р В Р’В Р Р†Р вЂљР’В Р В Р’В Р вЂ™Р’В Р В Р вЂ Р В РІР‚С™Р РЋРІвЂћСћР В Р’В Р В РІР‚В Р В Р’В Р Р†Р вЂљРЎв„ўР В Р Р‹Р РЋРЎв„ў reselecting")
                current_target = exploration.get_next_target(pose, coverage)
                current_target_steps = 0
        if current_target is None:
            robot.stop()
            if idle_steps == 1:
                print(f"[Controller] Exploration complete at step {step_count}")
                print(f"  Coverage: {coverage.coverage_percent():.1f}%")
            robot.step()
            continue

        cmd = navigation.compute_command(
            pose,
            current_target[0],
            current_target[1],
            sensor_data=sonar_data,
        )
        if escape_steps > 0:
            robot.apply_command(cmd)
            escape_steps -= 1
            if escape_steps == 0:
                current_target = None
            continue
        robot.apply_command(cmd)
        if navigation.is_target_reached(pose, current_target[0], current_target[1]):
            coverage.mark_visited(pose)
            targets_reached += 1
            exploration.reset_target()
            current_target = None
        if idle_steps > 100:
            print(f"  [Controller] STUCK at step {step_count} Р В Р’В Р вЂ™Р’В Р В Р’В Р Р†Р вЂљР’В Р В Р’В Р вЂ™Р’В Р В Р вЂ Р В РІР‚С™Р РЋРІвЂћСћР В Р’В Р В РІР‚В Р В Р’В Р Р†Р вЂљРЎв„ўР В Р Р‹Р РЋРЎв„ў picking escape target")
            _handle_stuck(robot, exploration, pose, coverage)
            exploration.add_blacklist(pose.x, pose.y)
            backup_dist = 2.0 + random.uniform(0.5, 2.0)
            current_target = (
                pose.x - backup_dist * math.cos(pose.theta),
                pose.y - backup_dist * math.sin(pose.theta),
            )
            escape_steps = 35
            idle_steps = 0
            _checkpoint = Pose(pose.x, pose.y, pose.theta)
            _checkpoint_steps = 0
        area_after = coverage.coverage_percent()
        if step_count - last_log_step >= cfg.log_interval:
            elapsed = time.time() - start_wall
            print(
                f"[{step_count:>5}] coverage={area_after:.1f}%  "
                f"targets={targets_reached}  "
                f"pose=({pose.x:.2f}, {pose.y:.2f}, {math.degrees(pose.theta):.0f}Р В Р’В Р вЂ™Р’В Р В Р вЂ Р В РІР‚С™Р Р†РІР‚С›РЎС›Р В Р’В Р Р†Р вЂљРІвЂћСћР В РІР‚в„ўР вЂ™Р’В°)  "
                f"Р В Р’В Р вЂ™Р’В Р В Р Р‹Р Р†Р вЂљРЎвЂќР В Р’В Р В РІР‚В Р В Р’В Р Р†Р вЂљРЎв„ўР В Р Р‹Р РЋРЎв„ўcoverage={area_after - area_before:.1f}%  "
                f"wall={elapsed:.0f}s"
            )
            last_log_step = step_count

        elapsed = time.time() - start_wall
    print(f"\n[Controller] Simulation ended after {step_count} steps ({elapsed:.0f}s)")
    print(f"  Final coverage: {coverage.coverage_percent():.1f}%")
    print(f"  Targets reached: {targets_reached}")

def _handle_stuck(
    robot: WebotsRobot,
    exploration: FrontierExploration,
    pose: Pose,
    coverage: CoverageMap,
) -> tuple[float, float]:
    import math

    robot.stop()
    sonars = robot.read_sonars()
    angles = sonars.angles
    dists = sonars.distances
    safest_angle = 0.0
    safest_dist = 0.0
    for a, d in zip(angles, dists):
        d = d if d < 10 else 10.0
        if d > safest_dist:
            safest_dist = d
            safest_angle = a
    escape_angle = pose.theta + safest_angle + random.uniform(-0.3, 0.3)
    dist = min(max(safest_dist * 0.6, 1.0), 3.0)
    escape_x = pose.x + dist * math.cos(escape_angle)
    escape_y = pose.y + dist * math.sin(escape_angle)
    exploration.last_target = (escape_x, escape_y)
    return (escape_x, escape_y)


if __name__ == "__main__":
    main()
