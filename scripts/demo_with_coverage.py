import math

from controllers.robot_mapper.models.coverage_map import CoverageMap
from controllers.robot_mapper.models.pose import Pose
from controllers.robot_mapper.strategies.exploration.greedy import (
    GreedyExploration,
)
from controllers.robot_mapper.strategies.exploration.random import (
    RandomExploration,
)
from controllers.robot_mapper.strategies.navigation.differential import (
    DifferentialDriveNavigation,
)


def run_demo(exploration_strategy, name: str, steps: int = 100):
    print(f"\n=== {name} ===")

    coverage = CoverageMap(resolution=0.2, map_size=200)
    navigation = DifferentialDriveNavigation(
        max_linear_speed=0.5,
        max_angular_speed=2.0,
        angle_tolerance=0.05,
        position_tolerance=0.1
    )
    strategy = exploration_strategy
    strategy.reset()

    pose = Pose(x=0.0, y=0.0, theta=0.0)
    coverage.mark_visited(pose)

    targets_reached = 0
    timestep = 0.032  # секунды

    for step in range(steps):
        target = strategy.get_next_target(pose, coverage)

        if target is None:
            print(f"Exploration complete at step {step}")
            break

        # Навигация к цели
        reached = False
        max_iterations = 500  # больше итераций на цель
        for _ in range(max_iterations):
            cmd = navigation.compute_command(pose, target[0], target[1])

            # Обновление позиции (ВСЕГДА обновляем угол, скорость может быть нулевой)
            pose.theta += cmd.angular_velocity * timestep
            pose.theta = math.atan2(math.sin(pose.theta), math.cos(pose.theta))

            # Линейное движение только если есть скорость
            if cmd.linear_velocity != 0:
                pose.x += cmd.linear_velocity * timestep * math.cos(pose.theta)
                pose.y += cmd.linear_velocity * timestep * math.sin(pose.theta)

            # Проверка достижения цели
            if navigation.is_target_reached(pose, target[0], target[1]):
                coverage.mark_visited(pose)
                targets_reached += 1
                reached = True
                if hasattr(strategy, 'reset_target'):
                    strategy.reset_target()
                break

        if not reached:
            # Принудительный сброс, чтобы не зависнуть
            if hasattr(strategy, 'reset_target'):
                strategy.reset_target()

        if step % 20 == 0 and step > 0:
            print(f"Step {step}: coverage = {coverage.coverage_percent():.4f}%, targets reached = {targets_reached}")

    print(f"Final coverage: {coverage.coverage_percent():.4f}%")
    print(f"Targets reached: {targets_reached}")
    return coverage.coverage_percent()


if __name__ == "__main__":
    print("Robot Exploration Demo")
    print("=" * 50)

    steps = 1000

    random_score = run_demo(RandomExploration(max_steps=steps), "Random Strategy", steps)
    greedy_score = run_demo(
        GreedyExploration(search_radius=5.0, min_target_distance=0.8),
        "Greedy Strategy",
        steps
    )

    print(f"\n{'=' * 50}")
    print("RESULTS:")
    print(f"  Random coverage: {random_score:.2f}%")
    print(f"  Greedy coverage: {greedy_score:.2f}%")
