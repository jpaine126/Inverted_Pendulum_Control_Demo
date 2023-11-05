# -*- coding: utf-8 -*-
import numpy as np
from scipy import signal


class Plant:
    """Plant class to handle physics and integration.

    All state matrices will be kept in discrete form. If dt is provided, the
    state matrices must be given in continuous form. dt is not used for the
    actual propagation because a constant dt is assumed.
    """

    def __init__(
        self,
        mass_cart,
        mass_arm,
        length_arm,
        friction,
        gravity,
        state,
    ):
        self.state = state
        self.mass_cart = mass_cart
        self.mass_arm = mass_arm
        self.length_arm = length_arm
        self.friction = friction
        self.gravity = gravity
        self.arm_moment = (1 / 3) * mass_arm * (length_arm**2)

    def derivative(self, state, force):
        """Calculate derivatives of the non-linear eqs of motion."""
        M = self.mass_cart
        m = self.mass_arm
        l = self.length_arm
        b = self.friction
        g = self.gravity
        I = self.arm_moment

        x_dot = state[1]
        theta = state[2] + np.pi
        theta_dot = state[3]

        A = np.array(
            [
                [M + m, m * l * np.cos(theta)],
                [m * l * np.cos(theta), I + m * l**2],
            ]
        )
        B = np.array(
            [
                force - b * x_dot + m * l * (theta_dot**2) * np.sin(theta),
                -m * g * l * np.sin(theta),
            ]
        )

        accels = np.linalg.inv(A) @ B
        return np.hstack((x_dot, accels[0], theta_dot, accels[1]))
