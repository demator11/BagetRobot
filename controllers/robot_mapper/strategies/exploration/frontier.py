import math
from typing import Optional, Tuple

import numpy as np

from ...models.coverage_map import CoverageMap
from ...models.obstacle_grid import FREE, UNKNOWN, OccupancyGrid
from ...models.pose import Pose


class FrontierExploration:

    def __init__(
        self,
        occupancy_grid: OccupancyGrid,
        min_frontier_size: int = 5,
        min_target_distance: float = 1.5,
        search_margin: int = 30,
    ):
        self.occupancy_grid = occupancy_grid
        self.min_frontier_size = min_frontier_size
        self.min_target_distance = min_target_distance
        self.search_margin = search_margin
        self.complete = False
        self.last_target: Optional[Tuple[float, float]] = None
        self.blacklist: list[Tuple[float, float]] = []

    def get_next_target(
        self,
        current_pose: Pose,
        coverage_map: CoverageMap,
    ) -> Optional[Tuple[float, float]]:
        grid = self.occupancy_grid.grid
        h, w = grid.shape

        cx, cy = self.occupancy_grid._world_to_cell(
            current_pose.x, current_pose.y
        )
        x_min = max(0, cx - self.search_margin)
        x_max = min(w, cx + self.search_margin)
        y_min = max(0, cy - self.search_margin)
        y_max = min(h, cy + self.search_margin)

        local_grid = grid[y_min:y_max, x_min:x_max]
        frontier_mask = self._find_frontier_mask(local_grid)

        if frontier_mask.any():
            clusters = self._cluster(frontier_mask)
            if clusters:
                return self._select_target(
                    clusters, current_pose, x_min, y_min
                )

        full_mask = self._find_frontier_mask(grid)
        if not full_mask.any():
            self.complete = True
            return None

        clusters = self._cluster(full_mask)
        if not clusters:
            self.complete = True
            return None

        return self._select_target(clusters, current_pose, 0, 0)

    def _find_frontier_mask(self, grid: np.ndarray) -> np.ndarray:
        free = (grid == FREE)
        unknown = (grid == UNKNOWN)

        up = np.pad(unknown[:-1, :], ((1, 0), (0, 0)), constant_values=False)
        down = np.pad(unknown[1:, :], ((0, 1), (0, 0)), constant_values=False)
        left = np.pad(unknown[:, :-1], ((0, 0), (1, 0)), constant_values=False)
        right = np.pad(unknown[:, 1:], ((0, 0), (0, 1)), constant_values=False)
        adj_unknown = up | down | left | right

        return free & adj_unknown

    def _cluster(self, mask: np.ndarray) -> list[np.ndarray]:
        h, w = mask.shape
        visited = np.zeros_like(mask, dtype=bool)
        clusters = []

        for y in range(h):
            for x in range(w):
                if not mask[y, x] or visited[y, x]:
                    continue
                cluster = []
                queue = [(y, x)]
                visited[y, x] = True
                while queue:
                    cy, cx = queue.pop(0)
                    cluster.append((cy, cx))
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            if dy == 0 and dx == 0:
                                continue
                            ny, nx = cy + dy, cx + dx
                            if 0 <= ny < h and 0 <= nx < w:
                                if mask[ny, nx] and not visited[ny, nx]:
                                    visited[ny, nx] = True
                                    queue.append((ny, nx))
                if len(cluster) >= self.min_frontier_size:
                    clusters.append(np.array(cluster))

        clusters.sort(key=len, reverse=True)
        return clusters

    def _select_target(
        self,
        clusters: list[np.ndarray],
        current_pose: Pose,
        x_offset: int,
        y_offset: int,
    ) -> Optional[Tuple[float, float]]:
        best_score = -1.0
        best_target = None

        for cluster in clusters[:8]:
            ys, xs = cluster[:, 0], cluster[:, 1]
            cy = float(np.mean(ys)) + y_offset
            cx = float(np.mean(xs)) + x_offset
            wx, wy = self.occupancy_grid._cell_to_world(
                int(cx), int(cy)
            )
            dist = math.hypot(wx - current_pose.x, wy - current_pose.y)
            if dist < self.min_target_distance:
                continue
            target_angle = math.atan2(
                wy - current_pose.y, wx - current_pose.x
            )
            angle_diff = target_angle - current_pose.theta
            angle_diff = math.atan2(
                math.sin(angle_diff), math.cos(angle_diff)
            )
            direction = max(math.cos(angle_diff), 0.0)
            blacklisted = any(
                math.hypot(wx - bx, wy - by) < 1.5
                for bx, by in self.blacklist
            )
            if blacklisted:
                continue
            size = len(cluster)
            score = size / max(dist, 0.5) * direction

            if score <= best_score:
                continue

            if not self.occupancy_grid.path_is_free(
                current_pose.x, current_pose.y, wx, wy
            ):
                if dist < 0.5:
                    continue
                reachable = False
                for angle_deg in range(0, 360, 15):
                    rad = math.radians(angle_deg)
                    px = wx + math.cos(rad) * 0.3
                    py = wy + math.sin(rad) * 0.3
                    if self.occupancy_grid.path_is_free(
                        current_pose.x, current_pose.y, px, py
                    ):
                        reachable = True
                        dist = math.hypot(
                            px - current_pose.x, py - current_pose.y
                        )
                        score = size / max(dist, 0.5) * direction
                        if score > best_score:
                            best_score = score
                            best_target = (px, py)
                        break
                if not reachable:
                    continue

            best_score = score
            best_target = (wx, wy)

        if best_target is not None:
            self.last_target = best_target
            return best_target

        for cluster in clusters:
            ys, xs = cluster[:, 0], cluster[:, 1]
            cy = float(np.mean(ys)) + y_offset
            cx = float(np.mean(xs)) + x_offset
            wx, wy = self.occupancy_grid._cell_to_world(int(cx), int(cy))
            dist = math.hypot(wx - current_pose.x, wy - current_pose.y)
            blacklisted = any(
                math.hypot(wx - bx, wy - by) < 1.5
                for bx, by in self.blacklist
            )
            if blacklisted:
                continue
            target_angle = math.atan2(
                wy - current_pose.y, wx - current_pose.x
            )
            angle_diff = target_angle - current_pose.theta
            angle_diff = math.atan2(
                math.sin(angle_diff), math.cos(angle_diff)
            )
            if math.cos(angle_diff) < 0.1:
                continue
            self.last_target = (wx, wy)
            return (wx, wy)

        return None

    def add_blacklist(self, x: float, y: float) -> None:
        self.blacklist.append((x, y))
        if len(self.blacklist) > 10:
            self.blacklist.pop(0)

    def reset_target(self) -> None:
        self.last_target = None

    def is_complete(self) -> bool:
        return self.complete

    def reset(self) -> None:
        self.complete = False
        self.last_target = None
