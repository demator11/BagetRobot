from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

from .models.coverage_map import CoverageMap
from .models.pose import Pose


class OccupancyGridVisualizer:
    """Визуализация состояния робота через matplotlib.

    Окно 1 — карта занятости (occupancy grid):
      - Чёрный (1.0) = стена
      - Белый (0.0) = свободно
      - Серый (0.5) = неизвестно
      - Зелёная точка = робот
      - Красный крест = текущая цель

    Окно 2 — карта покрытия:
      - Чёрный = не посещено
      - Зелёный = посещено 1 раз
      - Синий = посещено многократно
      - Линия = траектория робота
    """

    def __init__(
        self,
        map_size: int = 200,
        resolution: float = 0.2,
        update_interval: int = 5,
    ):
        self.map_size = map_size
        self.resolution = resolution
        self.update_interval = update_interval
        self._step = 0
        self._trajectory: List[Tuple[float, float]] = []

        half_extent = map_size / 2 * resolution

        # --- Window 1: Occupancy Grid ---
        self.fig1, self.ax1 = plt.subplots(figsize=(8, 8))
        self.fig1.canvas.manager.set_window_title("Occupancy Grid")

        self._grid = np.full((map_size, map_size), 0.5)
        self._im = self.ax1.imshow(
            self._grid,
            cmap="gray",
            vmin=0,
            vmax=1,
            origin="lower",
            extent=[-half_extent, half_extent, -half_extent, half_extent],
        )

        (self._robot_dot,) = self.ax1.plot(
            [], [], "go", markersize=8, label="Robot"
        )
        (self._target_marker,) = self.ax1.plot(
            [], [], "rx", markersize=10, markeredgewidth=2, label="Target"
        )

        self.ax1.set_xlabel("X (m)")
        self.ax1.set_ylabel("Y (m)")
        self.ax1.set_title("Occupancy Grid")
        self.ax1.grid(True, alpha=0.3)
        self.ax1.legend(loc="upper right")
        self.ax1.set_aspect("equal")

        # --- Window 2: Coverage Heatmap ---
        self.fig2, self.ax2 = plt.subplots(figsize=(8, 8))
        self.fig2.canvas.manager.set_window_title("Coverage Heatmap")

        self._cmap = ListedColormap(["black", "green", "blue"])
        self._visit_grid = np.zeros((map_size, map_size), dtype=np.int32)
        self._visit_im = self.ax2.imshow(
            self._visit_grid,
            cmap=self._cmap,
            vmin=0,
            vmax=2,
            origin="lower",
            extent=[-half_extent, half_extent, -half_extent, half_extent],
        )

        (self._traj_line,) = self.ax2.plot(
            [], [], color="cyan", linewidth=2, alpha=0.9, label="Trajectory"
        )
        (self._robot_dot2,) = self.ax2.plot(
            [], [], "go", markersize=8, label="Robot"
        )

        self.ax2.set_xlabel("X (m)")
        self.ax2.set_ylabel("Y (m)")
        self.ax2.set_title("Coverage Heatmap")
        self.ax2.grid(True, alpha=0.3)
        self.ax2.legend(loc="upper right")
        self.ax2.set_aspect("equal")

        plt.ion()
        plt.show(block=False)

    def update(
        self,
        pose: Pose,
        coverage_map: CoverageMap,
        target: Optional[Tuple[float, float]] = None,
        occupancy_grid: Optional[np.ndarray] = None,
    ):
        self._step += 1
        if self._step % self.update_interval != 0:
            return

        self._trajectory.append((pose.x, pose.y))

        # --- Window 1 ---
        self._render_map(coverage_map, occupancy_grid)
        self._im.set_data(self._grid)
        self._robot_dot.set_data([pose.x], [pose.y])

        if target is not None:
            self._target_marker.set_data([target[0]], [target[1]])
        else:
            self._target_marker.set_data([], [])

        self.fig1.canvas.draw()
        self.fig1.canvas.flush_events()

        # --- Window 2 ---
        self._visit_grid = coverage_map.get_visit_grid(self.map_size)
        self._visit_grid = np.clip(self._visit_grid, 0, 2)
        self._visit_im.set_data(self._visit_grid)

        if len(self._trajectory) > 1:
            xs, ys = zip(*self._trajectory)
            self._traj_line.set_data(xs, ys)

        self._robot_dot2.set_data([pose.x], [pose.y])

        self.fig2.canvas.draw()
        self.fig2.canvas.flush_events()

    def _render_map(
        self,
        coverage_map: CoverageMap,
        occupancy_grid: Optional[np.ndarray] = None,
    ):
        if occupancy_grid is not None:
            self._grid = occupancy_grid.copy()
            free = self._grid == 0.0
            self._grid[free] = 0.5
            for (cx, cy) in coverage_map.visits:
                if 0 <= cx < self.map_size and 0 <= cy < self.map_size:
                    if self._grid[cy, cx] != 1.0:
                        self._grid[cy, cx] = 0.0
        else:
            self._grid.fill(0.5)
            for (cx, cy) in coverage_map.visits:
                if 0 <= cx < self.map_size and 0 <= cy < self.map_size:
                    self._grid[cy, cx] = 0.0

    def close(self):
        plt.close(self.fig1)
        plt.close(self.fig2)
