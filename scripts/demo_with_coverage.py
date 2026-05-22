import math
import random

from controllers.robot_mapper.models.coverage_map import CoverageMap
from controllers.robot_mapper.models.obstacle_grid import OccupancyGrid
from controllers.robot_mapper.models.pose import Pose
from controllers.robot_mapper.strategies.exploration.greedy import (
    GreedyExploration,
)
from controllers.robot_mapper.strategies.navigation.differential import (
    DifferentialDriveNavigation,
)
from controllers.robot_mapper.visualizer import OccupancyGridVisualizer

SPAWN_RADIUS = 10.0

def generate_walls(grid: OccupancyGrid, count: int = 5, max_size: float = 2.0):
    """Разместить случайные прямоугольные стены в радиусе 10 м от старта."""
    for _ in range(count):
        cx = random.uniform(-SPAWN_RADIUS, SPAWN_RADIUS)
        cy = random.uniform(-SPAWN_RADIUS, SPAWN_RADIUS)
        w = random.uniform(0.5, max_size)
        h = random.uniform(0.5, max_size)
        grid.add_rectangle(cx, cy, w, h)

    
    for dx in range(-8, 9):
        for dy in range(-8, 9):
            x = dx * grid.resolution
            y = dy * grid.resolution
            cx, cy = grid._world_to_cell(x, y)
            if 0 <= cx < grid.map_size and 0 <= cy < grid.map_size:
                grid.grid[cy, cx] = 0.0

    
    grid.add_rectangle(2.2, 0, 0.4, 2.0)

def run_demo(exploration_strategy, name: str, steps: int = 200):
    print(f"\n=== {name} ===")

    obstacle_grid = OccupancyGrid(resolution=0.2, map_size=75)
    generate_walls(obstacle_grid)

    coverage = CoverageMap(resolution=0.2, map_size=75)
    navigation = DifferentialDriveNavigation(
        max_linear_speed=0.5,
        max_angular_speed=2.0,
        angle_tolerance=0.05,
        position_tolerance=0.1,
        obstacle_grid=obstacle_grid,
    )
    strategy = exploration_strategy
    strategy.reset()

    vis = OccupancyGridVisualizer(
        map_size=coverage.map_size,
        resolution=coverage.resolution,
        update_interval=10,
    )

    pose = Pose(x=0.0, y=0.0, theta=0.0)
    coverage.mark_visited(pose)

    targets_reached = 0
    timestep = 0.032

    for step in range(steps):
        target = strategy.get_next_target(pose, coverage)

        if target is None:
            print(f"Exploration complete at step {step}")
            break

        reached = False
        max_iterations = 500
        for _ in range(max_iterations):
            cmd = navigation.compute_command(pose, target[0], target[1])

            pose.theta += cmd.angular_velocity * timestep
            pose.theta = math.atan2(math.sin(pose.theta), math.cos(pose.theta))

            next_x = pose.x + cmd.linear_velocity * timestep * math.cos(pose.theta)
            next_y = pose.y + cmd.linear_velocity * timestep * math.sin(pose.theta)

            if cmd.linear_velocity != 0 and obstacle_grid.is_free(next_x, next_y):
                pose.x = next_x
                pose.y = next_y

            coverage.mark_visited(pose)
            vis.update(pose, coverage, target, obstacle_grid.to_numpy())

            if navigation.is_target_reached(pose, target[0], target[1]):
                coverage.mark_visited(pose)
                targets_reached += 1
                reached = True
                if hasattr(strategy, "reset_target"):
                    strategy.reset_target()
                break

        if not reached:
            if hasattr(strategy, "reset_target"):
                strategy.reset_target()

        if step % 20 == 0 and step > 0:
            print(
                f"Step {step}: coverage = {coverage.coverage_percent():.4f}%, "
                f"targets reached = {targets_reached}"
            )

    print(f"Final coverage: {coverage.coverage_percent():.4f}%")
    print(f"Targets reached: {targets_reached}")
    vis.close()
    return coverage.coverage_percent()

if __name__ == "__main__":
    print("Robot Exploration Demo")
    print("=" * 50)

    steps = 1000

    greedy_score = run_demo(
        GreedyExploration(search_radius=5.0, min_target_distance=0.8),
        "Greedy Strategy",
        steps,
    )

    # greedy_score = run_demo(
    #     RandomExploration(),
    #     "Greedy Strategy",
    #     steps,
    # )

    print(f"\n{'=' * 50}")
    print("RESULTS:")
    print(f"  Greedy coverage: {greedy_score:.2f}%")
