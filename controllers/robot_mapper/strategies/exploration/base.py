from typing import Optional, Protocol, Tuple

import numpy as np

from ...models.pose import Pose


class ExplorationStrategy(Protocol):
    """Протокол стратегии исследования."""

    def get_next_target(
            self,
            current_pose: Pose,
            occupancy_grid: np.ndarray,
    ) -> Optional[Tuple[float, float]]:
        ...

    def is_complete(self) -> bool:
        ...

    def reset(self) -> None:
        ...
