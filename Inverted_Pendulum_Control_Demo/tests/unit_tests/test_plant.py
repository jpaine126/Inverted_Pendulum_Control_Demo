import numpy as np
import pytest
from numpy.testing import assert_allclose

from Inverted_Pendulum_Control_Demo.plant import InvertedPendulum

DEFAULT_PARAMS = dict(
    mass_cart=0.5,
    mass_arm=0.2,
    length_arm=0.3,
    friction=0.1,
    gravity=9.8,
)


@pytest.fixture
def plant():
    state = np.array([0.0, 0.0, 0.0, 0.0])
    return InvertedPendulum(state=state, **DEFAULT_PARAMS)


class TestInvertedPendulumInit:
    def test_attributes_stored(self, plant):
        assert plant.mass_cart == 0.5
        assert plant.mass_arm == 0.2
        assert plant.length_arm == 0.3
        assert plant.cart_friction == 0.1
        assert plant.gravity == 9.8
        assert_allclose(plant.state, np.zeros(4))

    def test_arm_moment_formula(self, plant):
        """arm_moment = (1/3) * m * l^2."""
        assert_allclose(plant.arm_moment, (1 / 3) * 0.2 * 0.3**2)

    @pytest.mark.parametrize(
        "mass_arm, length_arm, expected_moment",
        [
            (0.2, 0.3, (1 / 3) * 0.2 * 0.3**2),
            (1.0, 1.0, 1 / 3),
            (2.0, 3.0, (1 / 3) * 2.0 * 9.0),
        ],
    )
    def test_arm_moment_for_various_params(self, mass_arm, length_arm, expected_moment):
        p = InvertedPendulum(
            mass_cart=1.0,
            mass_arm=mass_arm,
            length_arm=length_arm,
            friction=0.0,
            gravity=0.0,
            state=np.zeros(4),
        )
        assert_allclose(p.arm_moment, expected_moment)

    def test_implements_protocol_shape(self, plant):
        """InvertedPendulum exposes the PlantProtocol surface (derivative + linear_state_space)."""
        assert callable(plant.derivative)
        assert callable(plant.linear_state_space)


class TestLinearStateSpace:
    def test_matrix_shapes(self, plant):
        A, B, C, D = plant.linear_state_space()
        assert A.shape == (4, 4)
        assert B.shape == (4, 1)
        assert C.shape == (2, 4)
        assert D.shape == (2, 1)

    def test_C_selects_position_and_angle(self, plant):
        """C should be [[1, 0, 0, 0], [0, 0, 1, 0]] (measure x and theta)."""
        _, _, C, _ = plant.linear_state_space()
        assert_allclose(C, np.array([[1, 0, 0, 0], [0, 0, 1, 0]]))

    def test_D_is_zero(self, plant):
        _, _, _, D = plant.linear_state_space()
        assert_allclose(D, np.zeros((2, 1)))

    def test_A_structure(self, plant):
        """A has the canonical form [[0,1,0,0],[0,_,_,0],[0,0,0,1],[0,_,_,0]]."""
        A, _, _, _ = plant.linear_state_space()
        assert_allclose(A[0], [0, 1, 0, 0])
        assert_allclose(A[2], [0, 0, 0, 1])
        # First and third rows of A have zeros in cols 0, 2 (and 3 / 0 resp.)

    def test_B_structure(self, plant):
        """B has the form [[0], [B2], [0], [B4]] - force only affects x and theta accels."""
        _, B, _, _ = plant.linear_state_space()
        assert B[0, 0] == 0
        assert B[2, 0] == 0

    def test_A_B_known_values_for_default_params(self, plant):
        """Verify the closed-form values of A and B for default physical params."""
        M, m, l, b, g = 0.5, 0.2, 0.3, 0.1, 9.8
        I = (1 / 3) * m * l**2
        P = I * (M + m) + M * m * l**2
        A22 = -((I + m * l**2) * b) / P
        A23 = (m**2 * g * l**2) / P
        A42 = -(m * l * b) / P
        A43 = (m * g * l * (M + m)) / P
        B2 = (I + m * l**2) / P
        B4 = (m * l) / P

        A, B, _, _ = plant.linear_state_space()
        assert_allclose(
            A,
            np.array(
                [
                    [0, 1, 0, 0],
                    [0, A22, A23, 0],
                    [0, 0, 0, 1],
                    [0, A42, A43, 0],
                ]
            ),
        )
        assert_allclose(B, np.array([[0], [B2], [0], [B4]]))


class TestDerivative:
    def test_returns_4_vector(self, plant):
        deriv = plant.derivative(np.array([0.0, 0.1, 0.0, 0.0]), 0.0)
        assert deriv.shape == (4,)

    def test_at_upright_equilibrium_returns_zeros(self, plant):
        """At state[2]=0 (upright, since derivative adds pi internally), zero
        velocity and zero force yields zero derivatives.
        """
        deriv = plant.derivative(np.zeros(4), 0.0)
        assert_allclose(deriv, np.zeros(4), atol=1e-10)

    def test_x_dot_passthrough(self, plant):
        """derivative[0] is the velocity state[1]."""
        deriv = plant.derivative(np.array([0.5, 1.5, 0.0, 0.0]), 0.0)
        assert deriv[0] == 1.5

    def test_theta_dot_passthrough(self, plant):
        """derivative[2] is the angular velocity state[3]."""
        deriv = plant.derivative(np.array([0.0, 0.0, 0.2, 0.7]), 0.0)
        assert deriv[2] == 0.7

    def test_positive_force_produces_positive_x_accel(self, plant):
        """A forward force on the upright pendulum accelerates it forward."""
        deriv = plant.derivative(np.array([0.0, 0.0, 0.0, 0.0]), 10.0)
        assert deriv[1] > 0

    def test_zero_force_at_equilibrium_has_zero_theta_accel(self, plant):
        deriv = plant.derivative(np.array([0.0, 0.0, 0.0, 0.0]), 0.0)
        assert deriv[3] == pytest.approx(0.0, abs=1e-10)

    def test_tilted_pendulum_has_nonzero_theta_accel(self, plant):
        """A pendulum away from upright (state[2] != 0) accelerates under gravity."""
        deriv = plant.derivative(np.array([0.0, 0.0, 0.5, 0.0]), 0.0)
        assert deriv[3] != 0

    def test_state_attribute_is_not_mutated_by_derivative(self, plant):
        """derivative should be a pure function of its args, not self.state."""
        original_state = plant.state.copy()
        plant.derivative(np.array([1.0, 2.0, 3.0, 4.0]), 5.0)
        assert_allclose(plant.state, original_state)
