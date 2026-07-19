import numpy as np
import pytest

from Inverted_Pendulum_Control_Demo.primitives import controllers

RAMP_TRAJECTORY = np.arange(0.0, 10.0, 0.1)
CONST_TRAJECTORY = np.full_like(RAMP_TRAJECTORY, 1.0)

CONTROLLER_TEST_CASES = [RAMP_TRAJECTORY]


class TestPID:
    def test_update_p(self):
        """Test update method proportional logic only"""
        pid = controllers.PID(1, 0, 0, set_point=0)
        # traj == set point, no force
        this_point = 0
        force = pid.update(this_point)
        assert isinstance(force, (int, float))
        assert force == 0
        assert pid.last_error == 0

        # check at error == -1
        this_point = 1
        expected_error = -this_point
        force = pid.update(this_point)
        assert force == -this_point
        assert pid.last_error == expected_error

    def test_update_i(self):
        """Test update method integrator logic only"""
        pid = controllers.PID(0, 1, 0, set_point=0, dt=1)
        # traj == set point, no force
        this_point = 0
        force = pid.update(this_point)
        assert isinstance(force, (int, float))
        assert force == 0
        assert pid.last_error == 0

        # check at error == -1
        this_point = 1
        expected_error = -this_point
        force = pid.update(this_point)
        assert force == -this_point
        assert pid.last_error == expected_error

    def test_update_d(self):
        """Test update method derivative logic only"""
        pid = controllers.PID(0, 0, 1, set_point=0, dt=1)
        # traj == set point, no force
        this_point = 0
        force = pid.update(this_point)
        assert isinstance(force, (int, float))
        assert force == 0
        assert pid.last_error == 0

        # check at error == -1
        this_point = 1
        expected_error = -this_point
        force = pid.update(this_point)
        assert force == -this_point
        assert pid.last_error == expected_error

    @pytest.mark.parametrize(
        "point",
        [1, 10, 1000],
    )
    def test_antiwindup_1(self, point):
        """Test antiwind up functionality for type 1"""
        pid = controllers.PID(0, 1, 0, set_point=0, antiwindupmode=1, control_limit=10)
        force = pid.update(point)
        assert abs(force) == min(abs(point), pid.control_limit)


class TestLQR:
    @pytest.mark.parametrize(
        "state",
        [
            np.array([1]),
            np.array([1, 1]),
            np.array([1, 1, 1]),
        ],
    )
    def test_update(self, state):
        lqr = controllers.LQR(np.atleast_2d(state))
        force = lqr.update(state.reshape((-1, 1)))
        assert np.isscalar(force)

    def test_update_uses_K_gain(self):
        """control_force = -sum(states[i, 0] * K[0, i])."""
        K = np.array([[1.0, 2.0, 3.0, 4.0]])
        lqr = controllers.LQR(K)
        states = np.array([[1.0], [1.0], [1.0], [1.0]])
        force = lqr.update(states)
        assert force == -10.0

    def test_max_input_off_no_clamp(self):
        """With max_input_on=False (default), large forces pass through."""
        K = np.array([[100.0]])
        lqr = controllers.LQR(K, max_input=5, max_input_on=False)
        force = lqr.update(np.array([[1.0]]))
        assert force == -100.0

    def test_max_input_on_below_threshold(self):
        """Small forces are unaffected by saturation."""
        K = np.array([[2.0]])
        lqr = controllers.LQR(K, max_input=5, max_input_on=True)
        force = lqr.update(np.array([[1.0]]))
        assert force == -2.0

    def test_max_input_on_clamps_positive_force(self):
        """Raw force above max_input is clamped; sign preserved via copysign.

        With state > 0 and K > 0, the raw internal control_force is positive,
        so copysign(max_input, +) keeps it positive, and the returned value
        (negated) is -max_input.
        """
        K = np.array([[100.0]])
        lqr = controllers.LQR(K, max_input=5, max_input_on=True)
        force = lqr.update(np.array([[1.0]]))
        assert force == -5.0

    def test_max_input_on_clamps_negative_force(self):
        """Raw force below -max_input is clamped; sign preserved via copysign.

        With state < 0 and K > 0, the raw internal control_force is negative,
        so copysign(max_input, -) makes it -max_input, and the returned value
        (negated) is +max_input.
        """
        K = np.array([[100.0]])
        lqr = controllers.LQR(K, max_input=5, max_input_on=True)
        force = lqr.update(np.array([[-1.0]]))
        assert force == 5.0

    @pytest.mark.parametrize(
        "max_input",
        [1, 5, 10],
    )
    def test_max_input_clamps_to_provided_limit(self, max_input):
        """Clamped force magnitude equals max_input regardless of raw value."""
        K = np.array([[1000.0]])
        lqr = controllers.LQR(K, max_input=max_input, max_input_on=True)
        force = lqr.update(np.array([[1.0]]))
        assert abs(force) == max_input
