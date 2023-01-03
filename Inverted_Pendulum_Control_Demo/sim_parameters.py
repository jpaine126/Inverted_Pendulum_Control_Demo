import control
import numpy as np
import pandas as pd
import param
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from scipy import signal

from .controllers import LQR, PID
from .main_sim import MainSim
from .observers import KalmanFilter, PassThroughObserver
from .plant import Plant

control_schemes = ["LQR", "PID", "None"]
observer_schemes = ["Kalman Filter", "None"]


class ControlDemoParam(param.Parameterized):
    """Parameters for the inverted pendulum control dashboard."""

    control_scheme = param.Selector(control_schemes, "LQR")
    observer_scheme = param.Selector(observer_schemes, "Kalman Filter")
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

    lqr_q = param.DataFrame(
        pd.DataFrame(
            np.diagflat([5000, 0, 100, 0]), index=pd.RangeIndex(0, 4, name="Q")
        ),
    )
    # lqr_r = param.Number(10)
    lqr_r = param.DataFrame(
        pd.DataFrame([10], index=pd.RangeIndex(0, 1, name="R")),
    )

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
    theta_initial = param.Number(0.0, bounds=(-180.0, 180.0))
    theta_dot_initial = param.Number(0.5, bounds=(-100.0, 100.0))

    t_final = param.Number(10, bounds=(0.0, 50))
    dt_plant = param.Number(0.001, bounds=(0.001, 0.5))
    dt_control = param.Number(0.01, bounds=(0.001, 0.5))

    # derived params

    arm_moment = param.Number(np.nan)

    plant_A = param.Array(np.array([]))

    plant_B = param.Array(np.array([]))

    @param.depends("mass_arm", "length_arm", watch=True, on_init=True)
    def calc_arm_moment(self):
        P = (
            1
            / 3
            * self.param.inspect_value("mass_arm")
            * (self.param.inspect_value("length_arm") ** 2)
        )
        self.param.set_param(arm_moment=P)

    @param.depends(
        "arm_moment",
        "mass_cart",
        "cart_friction",
        "gravity",
        watch=True,
        on_init=True,
    )
    def calc_plant(self):
        arm_moment = self.param.inspect_value("arm_moment")
        mass_cart = self.param.inspect_value("mass_cart")
        mass_arm = self.param.inspect_value("mass_arm")
        length_arm = self.param.inspect_value("length_arm")
        cart_friction = self.param.inspect_value("cart_friction")
        gravity = self.param.inspect_value("gravity")

        P = arm_moment * (mass_cart + mass_arm) + (
            mass_cart * mass_arm * length_arm**2
        )
        A22 = (-(arm_moment + mass_arm * (length_arm**2)) * cart_friction) / P
        A23 = ((mass_arm**2) * gravity * (length_arm**2)) / P
        A42 = (-(mass_arm * length_arm * cart_friction)) / P
        A43 = (mass_arm * gravity * length_arm * (mass_cart + mass_arm)) / P

        A = np.array([[0, 1, 0, 0], [0, A22, A23, 0], [0, 0, 0, 1], [0, A42, A43, 0]])

        B2 = (arm_moment + mass_arm * (length_arm**2)) / P
        B4 = (mass_arm * length_arm) / P

        B = np.array([[0], [B2], [0], [B4]])

        self.param.set_param(plant_A=A, plant_B=B)

    def view(self):
        state = np.array(
            [
                self.x_initial,
                self.x_dot_initial,
                self.theta_initial,
                self.theta_dot_initial,
            ]
        ).reshape((4, 1))

        plant = Plant(
            self.plant_A,
            self.plant_B,
            self.plant_C,
            self.plant_D,
            state,
            dt=self.dt_plant,
        )

        # get observer and controller
        if self.observer_scheme.lower() == "kalman filter":
            # observer = KalmanFilter(
            #     parameters.F_kalman,
            #     parameters.B_kalman,
            #     parameters.H_kalman,
            #     parameters.Q_kalman,
            #     parameters.R_kalman,
            # )
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
        elif self.control_scheme.lower() == "lqr":
            K, _, _ = control.lqr(
                self.plant_A,
                self.plant_B,
                self.lqr_q,
                self.lqr_r,
            )
            controller = LQR(K)
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
                name="theta",
            ),
            row=1,
            col=1,
        )

        fig.add_trace(
            go.Scatter(
                x=sim.t_control,
                y=sim.state_history[:][3],
                name="theta_dot",
            ),
            row=1,
            col=1,
        )

        fig.add_trace(
            go.Scatter(
                x=sim.t_control,
                y=sim.control_force_history,
                name="Control Force",
            ),
            row=2,
            col=1,
        )

        fig.add_trace(
            go.Scatter(
                x=sim.t_control,
                y=sim.state_history[:][2],
                name="Error",
            ),
            row=2,
            col=1,
        )

        fig.add_trace(
            go.Scatter(
                x=sim.t_control,
                y=sim.state_history[:][2],
                name="Error",
            ),
            row=3,
            col=1,
        )

        return fig
