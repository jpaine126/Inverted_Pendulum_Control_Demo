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
        A,
        B,
        C,
        D,
        state,
        dt=None,
        propogation_type="euler",
    ):
        self.state = state

        if propogation_type.lower() == "euler":
            self.update = self._update_euler
        elif propogation_type.lower() == "rk4":
            self.update = self._update_rk4
        else:
            raise ValueError(
                f"Unsupported propogation type {propogation_type}."
            )
        if dt is not None:
            self._from_continuous(A, B, C, D, dt)
        else:
            self.A = A
            self.B = B
            self.C = C
            self.D = D

    def _from_continuous(self, A, B, C, D, dt):
        """Make state eqs from continuous form and dt."""
        inverse_pendulum_plant = signal.StateSpace(A, B, C, D)

        # Convert to discrete time
        inverse_pendulum_plant_d = inverse_pendulum_plant.to_discrete(dt)

        # Extract discrete state matrices
        self.A = inverse_pendulum_plant_d.A
        self.B = inverse_pendulum_plant_d.B
        self.C = inverse_pendulum_plant_d.C
        self.D = inverse_pendulum_plant_d.D

    def _update_euler(self, control_force=0):
        """Propogate plant by one step, eulers method."""
        self.state = np.matmul(self.A, self.state) + self.B * control_force

    def _update_rk4(self, control_force=0):
        """Propogate plant by one step, rk4 method."""
        raise NotImplementedError("rk4 not implemented")

    def update():
        """Plant update place holder method."""
        raise NotImplementedError
