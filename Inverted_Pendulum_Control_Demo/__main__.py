"""Run the control test bench dashboard."""


import logging

import panel as pn
import panel.widgets as pnw

from .sim_parameters import ControlDemoParam


def run_sim(event):
    plot.object = obj.view()


if __name__ == "__main__":
    logging.info("Starting dashboard")
    obj = ControlDemoParam()

    plot = pn.pane.Plotly(obj.view(), width=800, height=800)

    inputs = []

    inputs.append(pnw.Button(name="Run Sim", button_type="primary"))

    inputs[0].on_click(run_sim)

    inputs.append(
        pn.Card(
            pnw.RadioButtonGroup.from_param(obj.param.control_scheme),
            pnw.RadioButtonGroup.from_param(obj.param.observer_scheme),
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
            pnw.FloatSlider.from_param(obj.param.theta_initial),
            pnw.FloatSlider.from_param(obj.param.theta_dot_initial),
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

    inputs.append(
        pn.Card(
            pnw.FloatSlider.from_param(obj.param.pid_p),
            pnw.FloatSlider.from_param(obj.param.pid_i),
            pnw.FloatSlider.from_param(obj.param.pid_d),
            title="PID Params",
        )
    )

    inputs.append(
        pn.Card(
            pnw.DataFrame.from_param(obj.param.lqr_q, width=300),
            pnw.DataFrame.from_param(obj.param.lqr_r, width=300),
            title="LQR Params",
        )
    )

    inputs.append(
        pn.Card(
            pnw.DataFrame.from_param(obj.param.Q_kalman, width=300),
            pnw.DataFrame.from_param(obj.param.R_kalman, width=300),
            title="KF Params",
        )
    )

    dashboard = pn.Row(pn.Column(*inputs), plot)

    pn.serve(dashboard)
