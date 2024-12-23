import numpy as np

from . import ObserverTestSetup


class PassThroughObserver(ObserverTestSetup, setup_name="Pass Through Observer"):
    """Observer that simply returns the whole true state as the measurement."""

    params = dict()

    def __init__(self, *args, **kwargs): ...

    def update(self, control_force: float, state: np.ndarray) -> np.ndarray:
        return state
