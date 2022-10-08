# -*- coding: utf-8 -*-
"""
Created on Mon Aug 24 14:03:15 2020

@author: jpaine

"""

from math import copysign


class PID:
    """PID Controller Class.

    Antiwideup Modes:
        0 - No Antiwindup Scheme

        1 - Saturate the Integrator Variable
    """

    def __init__(
        self,
        K_P,
        K_I,
        K_D,
        dt=1,
        last_error=0,
        antiwindupmode=0,
        control_limit=10,
    ):
        self.set_point = 0
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.dt = dt
        self.last_error = last_error
        self.integrator = 0
        self.control_limit = control_limit
        self.antiwindup_mode = antiwindupmode
        self.control_force = 0

    def __str__(self):
        """Class print statement."""
        return f"PID({self.K_P=}, {self.K_I=}, {self.K_D=}, {self.dt=}, {self.last_error=})"

    def update(self, states):
        """Update method to calculate force from state."""
        error = self.set_point - states[2][0]
        integral = self.integrator + error * self.dt
        deriv = (error - self.last_error) / self.dt

        if self.antiwindup_mode == 0:
            control_force = (
                error * self.K_P + integral * self.K_I + deriv * self.K_D
            )

        elif self.antiwindup_mode == 1:
            if abs(integral) > self.control_limit:
                integral = copysign(self.control_limit, integral)
            control_force = (
                error * self.K_P + integral * self.K_I + deriv * self.K_D
            )

        self.last_error = error
        self.integrator = integral

        self.control_force = control_force

        return control_force
