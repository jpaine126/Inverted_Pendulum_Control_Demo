# -*- coding: utf-8 -*-
import math
import numpy as np
from numpy.random import random_sample


class MainSim:
    """Simulation main execution loop."""

    def __init__(
        self,
        dt_plant,
        dt_control,
        t_final,
        controller,
        observer,
        plant,
        measurement_noise,
        measurement_noise_value,
        initial_conditions,
    ):
        self.dt_plant = dt_plant
        self.dt_control = dt_control

        self.t_plant = np.arange(0, t_final, dt_plant)
        self.t_control = np.arange(0, t_final, dt_control)

        self.full_step = int(dt_control / dt_plant)
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

    def control_step(self, state, control_force):
        """Single sim control step."""
        if self.measure_noise:
            adjusted_state = state + get_noise(self.measurement_noise_value)
        else:
            adjusted_state = state

        measurement = np.matmul(self.plant.C, adjusted_state)
        measurement = self.observer.update(control_force, measurement)

        control_force = self.controller.update(measurement)

        self.record(state, adjusted_state, measurement, control_force)

    def run_sim(self):
        """Run whole sim."""
        # collect initial conditions
        state = self.plant.state
        control_force = self.controller.control_force

        # fir bigger control steps
        for i in range(0, self.steps):
            self.control_step(state, control_force)
            state = self.state_history[:, -1]
            control_force = self.control_force_history[-1]

            # for plant propogation in between control steps
            for i in range(0, self.full_step):
                self.plant.update(control_force)
                state = self.plant.state


def get_noise(noise_mag, size=(4, 1)):
    """Make a noise vector of a given size and magnitude."""
    return (2 * random_sample(size) - 1) * noise_mag
