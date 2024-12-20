from typing import Protocol
import numpy as np


class Observer(Protocol):
    def update(self, state: np.ndarray) -> np.ndarray:
        """Calcualte filtered states from measured states."""


class Controller(Protocol):
    def update(self, state: np.ndarray) -> float:
        """Calcualte output force from states."""
