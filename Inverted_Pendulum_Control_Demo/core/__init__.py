from typing import Protocol

import numpy as np


class Observer(Protocol):
    def update(self, control_force: float, state: np.ndarray) -> np.ndarray:
        """Calcualte filtered states from measured states and inputs."""


class Controller(Protocol):
    def update(self, state: np.ndarray) -> float:
        """Calcualte output force from states."""
