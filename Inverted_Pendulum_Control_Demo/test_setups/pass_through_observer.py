import numpy as np
import plotly.graph_objects as go

from . import ObserverTestSetup


class PassThroughObserver(ObserverTestSetup, setup_name="Pass Through Observer"):
    """Observer that simply returns the whole true state as the measurement."""

    params = dict()

    def __init__(self, *args, **kwargs):
        self.estimate_history: list = []
        self.t_history: list = []

    def update(
        self, control_force: float, state: np.ndarray, time: float
    ) -> np.ndarray:
        self.estimate_history.append(np.asarray(state).reshape((-1,)).copy())
        self.t_history.append(float(time))
        return state

    def plot(self):
        """Return traces of the (passed-through) measured states vs time.

        Only position (x) and angle (phi) are returned, mirroring the Kalman
        filter, so they overlay cleanly with the plant's true-state traces.
        """
        if not self.estimate_history:
            return []
        estimates = np.array(self.estimate_history)
        t = np.array(self.t_history)
        return [
            go.Scatter(x=t, y=estimates[:, 0], name="x (meas)"),
            go.Scatter(x=t, y=estimates[:, 2], name="phi (meas)"),
        ]
