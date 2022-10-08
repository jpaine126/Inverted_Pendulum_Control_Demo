# -*- coding: utf-8 -*-
"""
Created on Sat Oct  1 18:02:27 2022

@author: jpaine
"""
import param
import pandas as pd
import numpy as np

control_schemes = ["LQR", "PID", "None"]
observer_schemes = ["Kalman Filter", "None"]


class ControlDemoParam(param.Parameterized):
    """Parameters for the control dashboard."""

    control_scheme = param.Selector(control_schemes, "LQR")
    observer_scheme = param.Selector(observer_schemes, "Kalman Filter")
    include_noise = param.Boolean(True)
    noise_value = param.Number(0.002, bounds=(0.0, None))

    mass_cart = param.Number(0.5, bounds=(0.0, None))
    mass_arm = param.Number(0.2, bounds=(0.0, None))
    length_arm = param.Number(0.3, bounds=(0.0, None))
    cart_friction = param.Number(0.1, bounds=(0.0, None))
    gravity = param.Number(9.8, bounds=(0.0, None))

    pid_p = param.Number(100, bounds=(0.0, None))
    pid_i = param.Number(1, bounds=(0.0, None))
    pid_d = param.Number(5, bounds=(0.0, None))

    lqr_q = param.DataFrame(pd.DataFrame(np.diagflat([5000, 0, 100, 0])))
    lqr_r = param.Number(10)

    F_kalman = param.DataFrame(
        pd.DataFrame(
            [
                [1, -0.01, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, -0.01],
                [0, 0, 0, 1],
            ]
        )
    )

    B_kalman = param.DataFrame(pd.DataFrame([[0], [0.1], [0], [0.1]]))

    H_kalman = param.DataFrame(pd.DataFrame([[1, 0, 0, 0], [0, 0, 1, 0]]))

    Q_kalman = param.DataFrame(
        pd.DataFrame(
            [
                [0.01, 0, 0, 0],
                [0, 0.01, 0, 0],
                [0, 0, 0.01, 0],
                [0, 0, 0, 0.01],
            ]
        )
    )

    R_kalman = param.DataFrame(
        pd.DataFrame(
            [
                [(0.001**4) / 4, (0.001**3) / 2],
                [(0.001**3) / 2, 0.001**2],
            ]
        )
    )

    x_initial = param.Number(0.0, bounds=(-10.0, 10.0))
    x_dot_initial = param.Number(0.0, bounds=(-10.0, 10.0))
    theta_initial = param.Number(0.0, bounds=(-180.0, 180.0))
    theta_dot_initial = param.Number(0.5, bounds=(-100.0, 100.0))

    t_final = param.Number(10, bounds=(0.0, 50))
    dt_plant = param.Number(0.001, bounds=(0.001, 0.5))
    dt_control = param.Number(0.01, bounds=(0.001, 0.5))
