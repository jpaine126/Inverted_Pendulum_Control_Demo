import numpy as np

from ..core.observers import PassThroughObserver as PassThroughObserverPrimitive
from . import ObserverTestSetup


class PassThroughObserver(ObserverTestSetup, name="Pass Through Observer"):
    params = dict()

    def __init__(self, *args, **kwargs):
        self.observer = PassThroughObserverPrimitive()

    def update(self, control_force: float, state: np.ndarray) -> np.ndarray:
        return self.observer.update(control_force, state)
