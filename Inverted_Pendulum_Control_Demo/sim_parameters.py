import numpy as np
import param


class ControlDemoParam(param.Parameterized):
    """General parameters for the inverted pendulum simulation."""

    include_noise = param.Boolean(True)
    noise_value = param.Number(0.002, bounds=(0.0, None))

    discretize_bin_size = param.Array(np.array([[0.0], [0.0], [0.0], [0.0]]))
    discretize_offset = param.Array(np.array([[0.0], [0.0], [0.0], [0.0]]))
    sensor_bias = param.Array(np.array([[0.0], [0.0], [0.0], [0.0]]))

    mass_cart = param.Number(0.5, bounds=(0.0, None))
    mass_arm = param.Number(0.2, bounds=(0.0, None))
    length_arm = param.Number(0.3, bounds=(0.0, None))
    cart_friction = param.Number(0.1, bounds=(0.0, None))
    gravity = param.Number(9.8, bounds=(0.0, None))

    x_initial = param.Number(0.0, bounds=(-10.0, 10.0))
    x_dot_initial = param.Number(0.0, bounds=(-10.0, 10.0))
    phi_initial = param.Number(0.5, bounds=(-np.pi, np.pi))
    phi_dot_initial = param.Number(0.0, bounds=(-10.0, 10.0))

    t_final = param.Number(10, bounds=(0.0, 50))
    dt_control = param.Number(0.01, bounds=(0.001, 0.5))
