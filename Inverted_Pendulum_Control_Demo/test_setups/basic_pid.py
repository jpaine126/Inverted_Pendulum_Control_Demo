import numpy as np
import pandas as pd
import plotly.graph_objects as go

from ..plant import PlantProtocol
from ..primitives.controllers import PID
from . import ControllerTestSetup


class BasicPID(ControllerTestSetup, name="Basic PID"):

    params = dict(
        pid_p=100.0,
        pid_i=1.0,
        pid_d=5.0,
    )

    def __init__(self, plant: PlantProtocol, sim_params, pid_p, pid_i, pid_d):
        dt = sim_params.dt_control
        self.controller = PID(pid_p, pid_i, pid_d, dt=dt)

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
