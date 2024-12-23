import numpy as np
import pandas as pd
import plotly.graph_objects as go
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

    def update(self, control_force: float, state: np.ndarray) -> np.ndarray:
        return self.observer.update(control_force, state)
