import numpy as np
import pandas as pd
from plotly import graph_objects as go
from scipy import signal

from ..plant import PlantProtocol
from ..primitives.observers import KalmanFilter
from . import ObserverTestSetup


class BasicKalmanFilter(ObserverTestSetup, setup_name="Basic Kalman Filter"):

    params = dict(
        Q=pd.DataFrame(
            [
                [0.01, 0, 0, 0],
                [0, 0.01, 0, 0],
                [0, 0, 0.01, 0],
                [0, 0, 0, 0.01],
            ],
            index=pd.RangeIndex(0, 4, name="Q"),
        ),
        R=pd.DataFrame(
            [
                [0.1],
                [0.05],
            ],
            index=pd.RangeIndex(0, 2, name="R"),
        ),
    )

    def __init__(self, plant: PlantProtocol, sim_params, Q, R):
        A, B, C, D = plant.linear_state_space()
        inverse_pendulum_plant = signal.StateSpace(
            A,
            B,
            C,
            D,
        )
        inverse_pendulum_plant_d_kalman = inverse_pendulum_plant.to_discrete(
            sim_params.dt_control
        )

        # Extract discrete state matrices
        A_discrete_k = inverse_pendulum_plant_d_kalman.A
        B_discrete_k = inverse_pendulum_plant_d_kalman.B
        C_discrete_k = inverse_pendulum_plant_d_kalman.C
        D_discrete_k = inverse_pendulum_plant_d_kalman.D

        observer = KalmanFilter(
            A_discrete_k,
            B_discrete_k,
            C_discrete_k,
            Q,
            R,
        )
        observer.x_last = np.array(
            [
                sim_params.x_initial,
                sim_params.x_dot_initial,
                sim_params.phi_initial,
                sim_params.phi_dot_initial,
            ]
        ).reshape((-1, 1))
        observer.P_last = np.eye(np.size(A, 1)) * sim_params.noise_value**2

        self.observer = observer
        self.plant = plant
        self.estimate_history: list = []
        self.t_history: list = []

    def update(
        self, control_force: float, state: np.ndarray, time: float
    ) -> np.ndarray:
        estimate = self.observer.update(control_force, state)
        self.estimate_history.append(np.asarray(estimate).reshape((-1,)).copy())
        self.t_history.append(float(time))
        return estimate

    def plot(self):
        """Return figures of the estimated states and estimation errors vs time.

        Returns a dict with an ``"Estimates"`` figure (estimated x and phi) and,
        when the plant's true-state history is available, an ``"Errors"`` figure
        (true − estimated position and angle). An empty history yields an empty
        dict.
        """
        if not self.estimate_history:
            return {}
        estimates = np.array(self.estimate_history)
        t = np.array(self.t_history)
        est_fig = go.Figure(
            data=[
                go.Scatter(x=t, y=estimates[:, 0], name="x (est)"),
                go.Scatter(x=t, y=estimates[:, 2], name="phi (est)"),
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
