import math

import numpy as np
from numpy.random import random_sample
from scipy.integrate import solve_ivp

from .plant import PlantProtocol
from .test_setups import ControllerTestSetup, ObserverTestSetup


class MainSim:
    """Simulation main execution loop."""

    def __init__(
        self,
        dt_plant: float,
        dt_control: float,
        t_final: float,
        controller: ControllerTestSetup,
        observer: ObserverTestSetup,
        plant: PlantProtocol,
        measurement_noise: bool,
        measurement_noise_value: float,
        initial_conditions: np.ndarray,
    ):
        self.dt_control = dt_control

        self.t_control = np.arange(0, t_final, dt_control)

        self.steps = math.ceil(t_final / dt_control)

        self.controller = controller
        self.observer = observer
        self.plant = plant

        self.measure_noise = measurement_noise
        self.measurement_noise_value = measurement_noise_value

        self.initial_conditions = initial_conditions
        self.state = initial_conditions

        self.state_history = [[], [], [], []]  # real states
        self.adjusted_state_history = [[], [], [], []]  # states w/ noise

        self.measurement_history = [[], [], [], []]

        # Collector for control force
        self.control_force_history = []

    def record(self, real_state, adjusted_state, measurement, control_force):
        """Record history."""
        self.state_history = np.append(self.state_history, real_state, axis=1)
        self.adjusted_state_history = np.append(
            self.adjusted_state_history, adjusted_state, axis=1
        )
        self.measurement_history = np.append(
            self.measurement_history, measurement, axis=1
        )
        self.control_force_history = np.append(
            self.control_force_history, control_force
        )

    def control_step(self, state, control_force, time):
        """Single sim control step."""
        if self.measure_noise:
            adjusted_state = state + get_noise(self.measurement_noise_value)
        else:
            adjusted_state = state

        measurement = self.observer.update(control_force, adjusted_state)

        control_force = self.controller.update(measurement, time)

        self.record(state, adjusted_state, measurement, control_force)

    def run_sim(self):
        """Run whole sim."""
        # collect initial conditions
        state = self.plant.state
        control_force = 0

        # for bigger control steps
        for i, time in enumerate(self.t_control):
            self.control_step(state, control_force, time)
            state = self.state_history[:, -1]
            control_force = self.control_force_history[-1]

            # for plant propogation in between control steps
            def wrapper(t, y, force):
                deriv = self.plant.derivative(y, force)
                return deriv

            a = solve_ivp(
                wrapper,
                t_span=(0, self.dt_control),
                y0=state,
                args=(control_force,),
                method="LSODA",
                min_step=0.0001,
            )

            state = np.atleast_2d(a.y[:, -1]).T
            self.plant.state = state


def get_noise(noise_mag, size=(4, 1)):
    """Make a noise vector of a given size and magnitude."""
    return (2 * random_sample(size) - 1) * noise_mag
