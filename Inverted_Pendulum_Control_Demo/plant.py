# -*- coding: utf-8 -*-
from typing import Protocol

import numpy as np


class PlantProtocol(Protocol):
    def derivative(self, state: np.ndarray, force: np.ndarray) -> np.ndarray:
        """Derivatives of full EQs of motion. Used for higher fidelity sim."""

    def linear_state_space(self):
        """Linear state space equations in continuous form."""


class Plant(PlantProtocol):
    """Plant class to handle physics and integration.

    All state matrices will be kept in discrete form. If dt is provided, the
    state matrices must be given in continuous form. dt is not used for the
    actual propagation because a constant dt is assumed.
    """

    def derivative(self, state: np.ndarray, force: np.ndarray) -> np.ndarray:
        """Derivatives of full EQs of motion.

        Used for numerical integration for non-linear simulation.
        """

    def linear_state_space(self):
        """Linear state space equations in continuous form.

        Not used in simulatino loop. Available for controllers and observers that want
        to design to the linear model.
        """


class InvertedPendulum(PlantProtocol):
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
        self.cart_friction = friction
        self.gravity = gravity
        self.arm_moment = (1 / 3) * mass_arm * (length_arm**2)

    def derivative(self, state, force):
        M = self.mass_cart
        m = self.mass_arm
        l = self.length_arm
        b = self.cart_friction
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

    def linear_state_space(self):
        mass_cart = self.mass_cart
        mass_arm = self.mass_arm
        length_arm = self.length_arm
        cart_friction = self.cart_friction
        gravity = self.gravity
        arm_moment = (1 / 3) * mass_arm * (length_arm**2)

        P = arm_moment * (mass_cart + mass_arm) + (mass_cart * mass_arm * length_arm**2)
        A22 = (-(arm_moment + mass_arm * (length_arm**2)) * cart_friction) / P
        A23 = ((mass_arm**2) * gravity * (length_arm**2)) / P
        A42 = (-(mass_arm * length_arm * cart_friction)) / P
        A43 = (mass_arm * gravity * length_arm * (mass_cart + mass_arm)) / P

        A = np.array([[0, 1, 0, 0], [0, A22, A23, 0], [0, 0, 0, 1], [0, A42, A43, 0]])

        B2 = (arm_moment + mass_arm * (length_arm**2)) / P
        B4 = (mass_arm * length_arm) / P

        B = np.array([[0], [B2], [0], [B4]])

        C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])

        D = np.array([[0], [0]])

        return A, B, C, D
