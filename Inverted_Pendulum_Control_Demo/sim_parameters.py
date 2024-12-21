import control
import numpy as np
import pandas as pd
import param
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from scipy import signal

from .core.controllers import PID, NoControl
from .core.observers import KalmanFilter, PassThroughObserver
from .main_sim import MainSim
from .plant import Plant
from .test_setups.lqr1 import LQR1

control_schemes = ["LQR", "PID", "None"]
observer_schemes = ["Kalman Filter", "None"]


class ControlDemoParam(param.Parameterized):
    """Parameters for the inverted pendulum control dashboard."""

    include_noise = param.Boolean(True)
    noise_value = param.Number(0.002, bounds=(0.0, None))

    mass_cart = param.Number(0.5, bounds=(0.0, None))
    mass_arm = param.Number(0.2, bounds=(0.0, None))
    length_arm = param.Number(0.3, bounds=(0.0, None))
    cart_friction = param.Number(0.1, bounds=(0.0, None))
    gravity = param.Number(9.8, bounds=(0.0, None))

    plant_C = param.Array(np.array([[1, 0, 0, 0], [0, 0, 1, 0]]))

    plant_D = param.Array(np.array([[0], [0]]))

    pid_p = param.Number(100, bounds=(0.0, None))
    pid_i = param.Number(1, bounds=(0.0, None))
    pid_d = param.Number(5, bounds=(0.0, None))

    F_kalman = param.DataFrame(
        pd.DataFrame(
            [
                [1, -0.01, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, -0.01],
                [0, 0, 0, 1],
            ],
            index=pd.RangeIndex(0, 4, name="F"),
        ),
    )

    B_kalman = param.DataFrame(
        pd.DataFrame([[0], [0.1], [0], [0.1]], index=pd.RangeIndex(0, 4, name="B")),
    )

    H_kalman = param.DataFrame(
        pd.DataFrame([[1, 0, 0, 0], [0, 0, 1, 0]], index=pd.RangeIndex(0, 2, name="H")),
    )

    Q_kalman = param.DataFrame(
        pd.DataFrame(
            [
                [0.01, 0, 0, 0],
                [0, 0.01, 0, 0],
                [0, 0, 0.01, 0],
                [0, 0, 0, 0.01],
            ],
            index=pd.RangeIndex(0, 4, name="Q"),
        ),
    )

    R_kalman = param.DataFrame(
        pd.DataFrame(
            [
                [(0.001**4) / 4, (0.001**3) / 2],
                [(0.001**3) / 2, 0.001**2],
            ],
            index=pd.RangeIndex(0, 2, name="R"),
        ),
    )

    x_initial = param.Number(0.0, bounds=(-10.0, 10.0))
    x_dot_initial = param.Number(0.0, bounds=(-10.0, 10.0))
    phi_initial = param.Number(0.5, bounds=(-np.pi, np.pi))
    phi_dot_initial = param.Number(0.0, bounds=(-10.0, 10.0))

    t_final = param.Number(10, bounds=(0.0, 50))
    dt_plant = param.Number(0.001, bounds=(0.001, 0.5))
    dt_control = param.Number(0.01, bounds=(0.001, 0.5))

    def view(self):
        state = np.array(
            [
                self.x_initial,
                self.x_dot_initial,
                self.phi_initial,
                self.phi_dot_initial,
            ]
        ).reshape((4, 1))

        plant = Plant(
            self.mass_cart,
            self.mass_arm,
            self.length_arm,
            self.cart_friction,
            self.gravity,
            state,
        )

        # get observer and controller
        if self.observer_scheme.lower() == "kalman filter":
            inverse_pendulum_plant = signal.StateSpace(
                self.plant_A,
                self.plant_B,
                self.plant_C,
                self.plant_D,
            )
            inverse_pendulum_plant_d_kalman = inverse_pendulum_plant.to_discrete(
                self.dt_control
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
                self.Q_kalman,
                self.R_kalman,
            )
            observer.x_last = state
            observer.P_last = np.eye(np.size(self.plant_A, 1)) * self.noise_value**2
        elif self.observer_scheme.lower() in {"pass through", "none"}:
            observer = PassThroughObserver()
        else:
            raise ValueError(f"Unsupported observer '{self.observer_scheme}'.")

        if self.control_scheme.lower() == "pid":
            controller = PID(
                self.pid_p,
                self.pid_i,
                self.pid_d,
                self.dt_control,
                last_error=-state[2][0],
            )
            controller.last_error = -state[2][0]
        elif self.control_scheme.lower() == "LQR 1":
            controller = LQR1(
                plant,
                self.lqr_q,
                self.lqr_r,
            )
        elif self.control_scheme.lower() == "none":
            controller = NoControl()
        else:
            raise ValueError(f"Unsupported controller '{self.control_scheme}'.")

        # run sim
        sim = MainSim(
            self.dt_plant,
            self.dt_control,
            self.t_final,
            controller,
            observer,
            plant,
            self.include_noise,
            self.noise_value,
            state,
        )

        sim.run_sim()

        # make plots
        fig = make_subplots(rows=3, shared_xaxes="columns")
        fig.add_trace(
            go.Scatter(
                x=sim.t_control,
                y=sim.state_history[:][0],
                name="x",
            ),
            row=1,
            col=1,
        )

        fig.add_trace(
            go.Scatter(
                x=sim.t_control,
                y=sim.state_history[:][1],
                name="x_dot",
            ),
            row=1,
            col=1,
        )

        fig.add_trace(
            go.Scatter(
                x=sim.t_control,
                y=sim.state_history[:][2],
                name="phi",
            ),
            row=1,
            col=1,
        )

        fig.add_trace(
            go.Scatter(
                x=sim.t_control,
                y=sim.state_history[:][3],
                name="phi_dot",
            ),
            row=1,
            col=1,
        )

        position_error = sim.state_history[:][0] - sim.measurement_history[:][0]
        angle_error = sim.state_history[:][2] - sim.measurement_history[:][2]

        fig.add_trace(
            go.Scatter(
                x=sim.t_control,
                y=position_error,
                name="Position Error",
            ),
            row=2,
            col=1,
        )

        fig.add_trace(
            go.Scatter(
                x=sim.t_control,
                y=angle_error,
                name="Angle Error",
            ),
            row=3,
            col=1,
        )

        return fig
