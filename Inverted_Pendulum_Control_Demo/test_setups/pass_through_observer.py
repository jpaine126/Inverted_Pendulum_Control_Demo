import numpy as np
from plotly import graph_objects as go

from ..plant import PlantProtocol
from . import ObserverTestSetup


class PassThroughObserver(ObserverTestSetup, setup_name="Pass Through Observer"):
    """Observer that simply returns the whole true state as the measurement."""

    params = dict()

    def __init__(self, plant: PlantProtocol = None, sim_params=None, **kwargs):
        self.plant = plant
        self.estimate_history: list = []
        self.t_history: list = []

    def update(
        self, control_force: float, state: np.ndarray, time: float
    ) -> np.ndarray:
        self.estimate_history.append(np.asarray(state).reshape((-1,)).copy())
        self.t_history.append(float(time))
        return state

    def plot(self):
        """Return figures of the measured states and measurement errors vs time.

        Returns a dict with an ``"Estimates"`` figure (measured x and phi) and,
        when the plant's true-state history is available, an ``"Errors"`` figure
        (true − measured position and angle, i.e. the effective sensor
        noise/discretization/bias). An empty history yields an empty dict.
        """
        if not self.estimate_history:
            return {}
        estimates = np.array(self.estimate_history)
        t = np.array(self.t_history)
        est_fig = go.Figure(
            data=[
                go.Scatter(x=t, y=estimates[:, 0], name="x (meas)"),
                go.Scatter(x=t, y=estimates[:, 2], name="phi (meas)"),
            ]
        )
        est_fig.update_layout(xaxis_title="Time (s)", yaxis_title="State")
        figures = {"Estimates": est_fig}

        plant = getattr(self, "plant", None)
        plant_history = getattr(plant, "state_history", None)
        plant_t_history = getattr(plant, "t_history", None)
        if plant_history and plant_t_history:
            plant_states = np.array(plant_history)
            plant_t = np.array(plant_t_history)
            n = min(len(plant_states), len(estimates), len(plant_t))
            if n > 0:
                err_fig = go.Figure(
                    data=[
                        go.Scatter(
                            x=plant_t[:n],
                            y=plant_states[:n, 0] - estimates[:n, 0],
                            name="Position Error",
                        ),
                        go.Scatter(
                            x=plant_t[:n],
                            y=plant_states[:n, 2] - estimates[:n, 2],
                            name="Angle Error",
                        ),
                    ]
                )
                err_fig.update_layout(xaxis_title="Time (s)", yaxis_title="Error")
                figures["Errors"] = err_fig

        return figures
