from typing import Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np

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

        half_extent = map_size / 2 * resolution

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title("Occupancy Grid")

        self._grid = np.full((map_size, map_size), 0.5)
        self._im = self.ax.imshow(
            self._grid,
            cmap="gray",
            vmin=0,
            vmax=1,
            origin="lower",
            extent=[-half_extent, half_extent, -half_extent, half_extent],
        )

        (self._robot_dot,) = self.ax.plot([], [], "go", markersize=8, label="Robot")
        (self._target_marker,) = self.ax.plot(
            [], [], "rx", markersize=10, markeredgewidth=2, label="Target"
        )

        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("Occupancy Grid")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="upper right")
        self.ax.set_aspect("equal")

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

        if occupancy_grid is not None:
            self._grid = occupancy_grid
        else:
            self._render_from_coverage(coverage_map)

        self._im.set_data(self._grid)
        self._robot_dot.set_data([pose.x], [pose.y])

        if target is not None:
            self._target_marker.set_data([target[0]], [target[1]])
        else:
            self._target_marker.set_data([], [])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def _render_from_coverage(self, coverage_map: CoverageMap):
        self._grid.fill(0.5)
        for cx, cy in coverage_map.visited:
            if 0 <= cx < self.map_size and 0 <= cy < self.map_size:
                self._grid[cy, cx] = 0.0

    def close(self):
        plt.close(self.fig)
