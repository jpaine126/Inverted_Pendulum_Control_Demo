import control
import numpy as np
import pandas as pd
import param
import plotly.graph_objects as go

from ..core.controllers import LQR
from ..plant import PlantProtocol
from . import ControllerTestSetup


class LQR1(ControllerTestSetup, name="LQR 1"):

    params = dict(
        lqr_q=pd.DataFrame(
            np.diagflat([5000.0, 0.0, 100.0, 0.0]), index=pd.RangeIndex(0, 4, name="Q")
        ),
        lqr_r=pd.DataFrame([10.0], index=pd.RangeIndex(0, 1, name="R")),
    )

    def __init__(self, plant: PlantProtocol, lqr_q, lqr_r):
        plant_A, plant_B, *_ = plant.linear_state_space()
        K, _, _ = control.lqr(
            plant_A,
            plant_B,
            lqr_q,
            lqr_r,
        )
        self.controller = LQR(K)

        self.control_history = []

    def update(self, states, time):
        force = self.controller.update(states)
        self.control_history.append((time, force))
        return force

    def plot(self):
        plot_data = np.array(self.control_history)
        return go.Scatter(
            x=plot_data[:, 0],
            y=plot_data[:, 1],
            name="Control Force",
        )
