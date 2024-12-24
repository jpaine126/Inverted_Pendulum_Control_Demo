"""Run the control test bench dashboard."""

import logging

import numpy as np
import pandas as pd
import panel as pn
import panel.widgets as pnw
from plotly import graph_objects as go
from plotly.subplots import make_subplots

from .main_sim import MainSim
from .plant import InvertedPendulum
from .sim_parameters import ControlDemoParam
from .test_setups import TestSetup


def run_sim(event):
    state = np.array(
        [
            obj.x_initial,
            obj.x_dot_initial,
            obj.phi_initial,
            obj.phi_dot_initial,
        ]
    ).reshape((4, 1))

    plant = InvertedPendulum(
        obj.mass_cart,
        obj.mass_arm,
        obj.length_arm,
        obj.cart_friction,
        obj.gravity,
        state,
    )

    # get observer and controller
    selected_observer = TestSetup._implemented_observers[observer_scheme.value]
    needed_observer_params = selected_observer.params

    param_values = {
        p: all_observer_params[observer_scheme.value][p].value
        for p in needed_observer_params.keys()
    }
    observer = selected_observer(
        plant,
        obj,
        **param_values,
    )

    selected_controller = TestSetup._implemented_controllers[controller_scheme.value]
    needed_controller_params = selected_controller.params
    param_values = {
        p: all_controller_params[controller_scheme.value][p].value
        for p in needed_controller_params.keys()
    }
    controller = selected_controller(
        plant,
        obj,
        **param_values,
    )

    # run sim
    sim = MainSim(
        dt_control=obj.dt_control,
        t_final=obj.t_final,
        controller=controller,
        observer=observer,
        plant=plant,
        measurement_noise=obj.include_noise,
        measurement_noise_value=obj.noise_value,
        initial_conditions=state,
        sensor_discretize=obj.discretize_bin_size,
        sensor_discretize_offset=obj.discretize_offset,
        sensor_bias=obj.sensor_bias,
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
    if isinstance(param, pd.DataFrame):
        return pnw.Tabulator(param)
    elif isinstance(param, float):
        return pnw.FloatInput(value=param)
    elif isinstance(param, float):
        return pnw.IntInput(value=param)
    elif isinstance(param, bool):
        return pnw.Toggle(value=param)
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
            pnw.FloatSlider.from_param(obj.param.t_final),
            pnw.FloatInput.from_param(obj.param.dt_control),
            pnw.Toggle.from_param(obj.param.include_noise),
            pnw.FloatInput.from_param(obj.param.noise_value),
            pnw.ArrayInput.from_param(obj.param.discretize_bin_size),
            pnw.ArrayInput.from_param(obj.param.discretize_offset),
            pnw.ArrayInput.from_param(obj.param.sensor_bias),
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

    all_controller_params = {}
    controller_cards = []
    for name, controller in TestSetup._implemented_controllers.items():
        controller_params = generate_widgets_from_params(controller.params)
        all_controller_params[name] = controller_params
        controller = pn.Card(
            title=name,
        )
        controller_cards.append(controller)
        controller.extend(controller_params.values())
        inputs.append(controller)

    all_observer_params = {}
    observer_cards = []
    for name, observer in TestSetup._implemented_observers.items():
        observer_params = generate_widgets_from_params(observer.params)
        observer = pn.Card(
            title=name,
        )
        observer_cards.append(observer)
        if observer_params:
            all_observer_params[name] = observer_params
            observer.extend(observer_params.values())
            inputs.append(observer)

    def hide_all_but_one(card_list, selector):
        """Provide a function that hides all widgets in a list except the one whose
        title matches the value of the selector."""

        def hide_all_but_one_helper(event):
            for w in card_list:
                if selector.value == w.title:
                    w.visible = True
                else:
                    w.visible = False

        return hide_all_but_one_helper

    controller_scheme.param.watch(
        hide_all_but_one(controller_cards, controller_scheme),
        "value",
    )

    observer_scheme.param.watch(
        hide_all_but_one(observer_cards, observer_scheme),
        "value",
    )

    dashboard = pn.Row(pn.Column(*inputs), plot)

    pn.serve(dashboard)
