"""Run the control test bench dashboard."""

import logging

import numpy as np
import pandas as pd
import panel as pn
import panel.widgets as pnw
import param
from plotly import graph_objects as go
from plotly.subplots import make_subplots
from scipy import signal

from .main_sim import MainSim
from .plant import Plant
from .sim_parameters import ControlDemoParam
from .test_setups import TestSetup
from .test_setups.lqr1 import LQR1
from .test_setups.pass_through_observer import PassThroughObserver


def run_sim(event):
    state = np.array(
        [
            obj.x_initial,
            obj.x_dot_initial,
            obj.phi_initial,
            obj.phi_dot_initial,
        ]
    ).reshape((4, 1))

    plant = Plant(
        obj.mass_cart,
        obj.mass_arm,
        obj.length_arm,
        obj.cart_friction,
        obj.gravity,
        state,
    )

    # get observer and controller
    if observer_scheme.value == "kalman filter":
        inverse_pendulum_plant = signal.StateSpace(
            obj.plant_A,
            obj.plant_B,
            obj.plant_C,
            obj.plant_D,
        )
        inverse_pendulum_plant_d_kalman = inverse_pendulum_plant.to_discrete(
            obj.dt_control
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
            obj.Q_kalman,
            obj.R_kalman,
        )
        observer.x_last = state
        observer.P_last = np.eye(np.size(obj.plant_A, 1)) * obj.noise_value**2
    elif observer_scheme.value in {"Pass Through Observer", "none"}:
        observer = PassThroughObserver()
    else:
        raise ValueError(f"Unsupported observer '{obj.observer_scheme}'.")

    selected_controller = TestSetup._implemented_controllers[controller_scheme.value]
    needed_controller_params = selected_controller.params
    param_values = {
        p: controller_params[p].value for p in needed_controller_params.keys()
    }
    controller = selected_controller(
        plant,
        **param_values,
    )

    # run sim
    sim = MainSim(
        obj.dt_plant,
        obj.dt_control,
        obj.t_final,
        controller,
        observer,
        plant,
        obj.include_noise,
        obj.noise_value,
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

    plot.object = fig


def to_widget(param):
    if param.__class__ is pd.DataFrame:
        return pnw.Tabulator(param)
    else:
        raise TypeError(f"Param {param} of type {type(param)} is not supported")


def generate_widgets_from_params(params):
    return {k: to_widget(p) for k, p in params.items()}


if __name__ == "__main__":
    logging.info("Starting dashboard")
    obj = ControlDemoParam()

    plot = pn.pane.Plotly(go.Figure(), width=800, height=800)

    inputs = []

    inputs.append(pnw.Button(name="Run Sim", button_type="primary"))

    inputs[0].on_click(run_sim)

    all_observers = list(TestSetup._implemented_observers.keys())
    all_controllers = list(TestSetup._implemented_controllers.keys())

    controller_scheme = pnw.RadioButtonGroup(options=all_controllers)
    observer_scheme = pnw.RadioButtonGroup(options=all_observers)

    inputs.append(
        pn.Card(
            controller_scheme,
            observer_scheme,
            pnw.Toggle.from_param(obj.param.include_noise),
            pnw.FloatInput.from_param(obj.param.noise_value),
            pnw.FloatSlider.from_param(obj.param.t_final),
            pnw.FloatInput.from_param(obj.param.dt_plant),
            pnw.FloatInput.from_param(obj.param.dt_control),
            title="Sim Params",
        )
    )

    inputs.append(
        pn.Card(
            pnw.FloatSlider.from_param(obj.param.x_initial),
            pnw.FloatSlider.from_param(obj.param.x_dot_initial),
            pnw.FloatSlider.from_param(obj.param.phi_initial),
            pnw.FloatSlider.from_param(obj.param.phi_dot_initial),
            title="Initial Conditions",
        )
    )

    inputs.append(
        pn.Card(
            pnw.FloatSlider.from_param(obj.param.mass_cart),
            pnw.FloatSlider.from_param(obj.param.mass_arm),
            pnw.FloatSlider.from_param(obj.param.length_arm),
            pnw.FloatSlider.from_param(obj.param.cart_friction),
            pnw.FloatSlider.from_param(obj.param.gravity),
            title="Plant Params",
        )
    )

    for name, controller in TestSetup._implemented_controllers.items():
        controller_params = generate_widgets_from_params(controller.params)
        controllers = pn.Card(
            title=name,
        )
        controllers.extend(controller_params.values())
        inputs.append(controllers)

    for name, observer in TestSetup._implemented_observers.items():
        observer_params = generate_widgets_from_params(observer.params)
        if observer_params:
            observers = pn.Card(
                title=name,
            )
            observers.extend(observer_params.values())
            inputs.append(observers)

    # inputs.append(
    #     pn.Card(
    #         pnw.FloatSlider.from_param(obj.param.pid_p),
    #         pnw.FloatSlider.from_param(obj.param.pid_i),
    #         pnw.FloatSlider.from_param(obj.param.pid_d),
    #         title="PID Params",
    #     )
    # )

    # inputs.append(
    #     pn.Card(
    #         pnw.DataFrame.from_param(obj.param.lqr_q, width=300),
    #         pnw.DataFrame.from_param(obj.param.lqr_r, width=300),
    #         title="LQR Params",
    #     )
    # )

    # inputs.append(
    #     pn.Card(
    #         pnw.DataFrame.from_param(obj.param.Q_kalman, width=300),
    #         pnw.DataFrame.from_param(obj.param.R_kalman, width=300),
    #         title="KF Params",
    #     )
    # )

    dashboard = pn.Row(pn.Column(*inputs), plot)

    pn.serve(dashboard)
