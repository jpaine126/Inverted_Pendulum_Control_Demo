from math import copysign

from .. import Controller

class LQR(Controller):
    """LQR Controller Class.

    This class if for implementing a LQR controller, not for designing one.
    """

    def __init__(self, K, max_input=1, max_input_on=False):
        self.K = K
        self.max_input = max_input
        self.max_input_on = max_input_on
        self.control_force = 0

    def update(self, states):
        """Update method to calculate force from state."""
        control_force = 0
        for i in range(0, len(states)):
            control_force += states[i, 0] * self.K[0, i]
            # print(states[i,0])
            # print(self.K[0,1])

        if self.max_input_on:
            if abs(control_force) > self.max_input:
                control_force = copysign(self.max_input, control_force)

        self.control_force = -control_force

        return -control_force
