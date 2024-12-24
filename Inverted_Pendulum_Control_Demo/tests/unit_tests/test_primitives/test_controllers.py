import numpy as np
import pytest

from Inverted_Pendulum_Control_Demo.primitives import controllers

RAMP_TRAJECTORY = np.arange(0.0, 10.0, 0.1)
CONST_TRAJECTORY = np.full_like(RAMP_TRAJECTORY, 1.0)

CONTROLLER_TEST_CASES = [RAMP_TRAJECTORY]


class TestPID:
    def test_init(self):
        pid = controllers.PID(1, 1, 1)

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
