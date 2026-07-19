import numpy as np
import pytest
from numpy.testing import assert_allclose

from Inverted_Pendulum_Control_Demo.primitives import observers


@pytest.fixture
def identity_kf():
    """4-D identity Kalman filter: F=I, B=0, H=I, Q=0, R=0.1*I.

    With this filter the per-dimension update reduces to the scalar
    Kalman equation x_post = x_pri + K * (z - x_pri) where
    K = P_pri / (P_pri + R).
    """
    F = np.eye(4)
    B = np.zeros((4, 1))
    H = np.eye(4)
    Q = np.zeros((4, 4))
    R = np.eye(4) * 0.1
    return observers.KalmanFilter(F, B, H, Q, R)


class TestKalmanFilterInit:
    def test_matrices_stored(self):
        F = np.eye(4)
        B = np.zeros((4, 1))
        H = np.eye(4)
        Q = np.ones((4, 4))
        R = np.eye(4) * 2.0
        kf = observers.KalmanFilter(F, B, H, Q, R)
        assert_allclose(kf.F, F)
        assert_allclose(kf.B, B)
        assert_allclose(kf.H, H)
        assert_allclose(kf.Q, Q)
        assert_allclose(kf.R, R)

    def test_state_and_covariance_init_to_zero(self):
        kf = observers.KalmanFilter(
            np.eye(4), np.zeros((4, 1)), np.eye(4), np.eye(4), np.eye(4)
        )
        assert kf.x_last.shape == (4, 1)
        assert_allclose(kf.x_last, np.zeros((4, 1)))
        assert kf.P_last.shape == (4, 4)
        assert_allclose(kf.P_last, np.zeros((4, 4)))

    def test_repr_contains_matrices(self):
        kf = observers.KalmanFilter(
            np.eye(4), np.zeros((4, 1)), np.eye(4), np.eye(4), np.eye(4)
        )
        repr_str = repr(kf)
        assert "KF(" in repr_str
        assert "F=" in repr_str
        assert "B=" in repr_str


class TestKalmanFilterUpdate:
    def test_first_call_returns_zero_prediction(self, identity_kf):
        """With x_last=0, P_last=0, R>0: Kalman gain is 0, so update returns the
        a priori prediction, which is F @ x_last + B*u = 0.
        """
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        result = identity_kf.update(0.0, state)
        assert result.shape == (4, 1)
        assert_allclose(result, np.zeros((4, 1)))

    def test_first_call_stores_zero_state_and_covariance(self, identity_kf):
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        identity_kf.update(0.0, state)
        # K = 0, so x_post = 0 and P_post = (I - 0) @ 0 = 0
        assert_allclose(identity_kf.x_last, np.zeros((4, 1)))
        assert_allclose(identity_kf.P_last, np.zeros((4, 4)))

    def test_update_with_known_value(self, identity_kf):
        """Verify the scalar Kalman equation per dimension when H=I, F=I, B=0."""
        identity_kf.x_last = np.array([[1.0], [1.0], [1.0], [1.0]])
        identity_kf.P_last = np.eye(4) * 10.0
        state = np.array([[2.0], [2.0], [2.0], [2.0]])
        result = identity_kf.update(0.0, state)
        # K = P_pri / (P_pri + R) = 10 / 10.1
        # x_post = x_pri + K * (z - x_pri) = 1 + (10/10.1) * 1
        expected = np.full((4, 1), 1.0 + 10.0 / 10.1)
        assert_allclose(result, expected)

    def test_update_stores_posterior_state(self, identity_kf):
        identity_kf.x_last = np.array([[1.0], [1.0], [1.0], [1.0]])
        identity_kf.P_last = np.eye(4) * 10.0
        state = np.array([[2.0], [2.0], [2.0], [2.0]])
        result = identity_kf.update(0.0, state)
        assert_allclose(identity_kf.x_last, result)

    def test_update_stores_posterior_covariance(self, identity_kf):
        identity_kf.x_last = np.array([[1.0], [1.0], [1.0], [1.0]])
        identity_kf.P_last = np.eye(4) * 10.0
        identity_kf.update(0.0, np.array([[2.0], [2.0], [2.0], [2.0]]))
        # P_post = (I - K*H) @ P_pri = (1 - 10/10.1) * 10
        expected_p = np.eye(4) * (1.0 - 10.0 / 10.1) * 10.0
        assert_allclose(identity_kf.P_last, expected_p)

    def test_control_input_affects_prediction(self):
        """B*u should add to the a priori state prediction."""
        F = np.eye(4)
        B = np.ones((4, 1))
        H = np.eye(4)
        Q = np.zeros((4, 4))
        R = np.eye(4) * 0.1
        kf = observers.KalmanFilter(F, B, H, Q, R)
        kf.x_last = np.array([[1.0], [1.0], [1.0], [1.0]])
        kf.P_last = np.eye(4) * 10.0
        state = np.array([[1.0], [1.0], [1.0], [1.0]])
        result = kf.update(2.0, state)
        # x_pri = x_last + B*u = [1+2, ...] = [3, 3, 3, 3]
        # y_hat = z - x_pri = [1-3, ...] = [-2, -2, -2, -2]
        # x_post = x_pri + K * y_hat = 3 + (10/10.1) * (-2)
        expected = np.full((4, 1), 3.0 - 20.0 / 10.1)
        assert_allclose(result, expected)

    def test_high_process_noise_trusts_measurement(self, identity_kf):
        """With P >> R, the Kalman gain approaches 1 and x_post approaches z."""
        identity_kf.x_last = np.array([[0.0], [0.0], [0.0], [0.0]])
        identity_kf.P_last = np.eye(4) * 1e6
        state = np.array([[5.0], [5.0], [5.0], [5.0]])
        result = identity_kf.update(0.0, state)
        assert_allclose(result, state)

    def test_high_measurement_noise_ignores_measurement(self, identity_kf):
        """With R >> P, the Kalman gain approaches 0 and x_post approaches x_pri."""
        identity_kf.x_last = np.array([[1.0], [1.0], [1.0], [1.0]])
        identity_kf.P_last = np.eye(4) * 0.1
        identity_kf.R = np.eye(4) * 1e6
        state = np.array([[5.0], [5.0], [5.0], [5.0]])
        result = identity_kf.update(0.0, state)
        # K ~ 0, x_post ~ x_pri = x_last (within the tiny Kalman gain)
        assert_allclose(result, np.full((4, 1), 1.0), atol=1e-5)

    def test_repeated_calls_persist_state(self, identity_kf):
        """Two consecutive updates: the second uses the first call's posterior."""
        identity_kf.x_last = np.array([[1.0], [1.0], [1.0], [1.0]])
        identity_kf.P_last = np.eye(4) * 10.0
        state_a = np.array([[2.0], [2.0], [2.0], [2.0]])
        first = identity_kf.update(0.0, state_a)
        state_b = np.array([[2.0], [2.0], [2.0], [2.0]])
        second = identity_kf.update(0.0, state_b)
        # Second call uses the posterior of the first as x_last; result should move
        # further toward the measurement (2.0) but not past it.
        assert np.all(second >= first)
        assert np.all(second <= 2.0)
