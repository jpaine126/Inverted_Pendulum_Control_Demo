import numpy as np
import pandas as pd

from ..plant import PlantProtocol
from ..primitives.observers import KalmanFilter
from .kalman_filter_base import KalmanFilterBase


class CAKalmanFilter(KalmanFilterBase, setup_name="CA Kalman Filter"):
    """Constant-acceleration Kalman filter on a 6D internal state.

    Internal state is ``[x, x_dot, x_ddot, phi, phi_dot, phi_ddot]``; the plant's
    ``[x, x_dot, phi, phi_dot]`` sit at indices ``[0, 1, 3, 4]``. Two independent
    constant-acceleration kinematic blocks (one per axis) are propagated, with
    the known control force injected into the position/velocity states via the
    linearized force-to-acceleration gains.

    A subtle but critical modeling point: the acceleration *states* track only
    the **residual** acceleration (the state-dependent dynamics the CA kinematic
    model cannot predict — gravity, friction, nonlinearities), NOT the control
    force. The force's effect on position and velocity is injected directly via
    ``B`` (the standard double-integrator chain ``[½dt², dt]``), while
    ``B[accel] = 0`` so the force does not accumulate into the acceleration
    state. (Injecting the force into the acceleration state causes it to grow
    without bound under a constant force and destabilizes the closed loop.)
    The total acceleration is recovered as ``residual + gain*u`` for plotting
    and NEES (see ``update``).

    The process noise uses the structured white-noise-jerk covariance (Bar-Shalom
    CA model) so the filter's covariance is physically consistent.
    """

    plant_state_indices = [0, 1, 3, 4]
    internal_state_dim = 6
    state_names = ["x", "x_dot", "x_ddot", "phi", "phi_dot", "phi_ddot"]

    params = dict(
        # Jerk spectral density (process noise intensity) shared by both axes.
        # A scalar replaces the old diagonal 6x6 DataFrame: the structured 6x6
        # Q is built from this and dt in __init__. Larger values tell the filter
        # to trust the model less and the measurements more. The default is
        # sized for the primary use case (closed-loop balance, where phi stays
        # near 0 and the residual acceleration varies slowly); it gives a
        # 6-DOF NEES near the chi-squared mean of 6 for the default plant. In
        # open-loop free-fall the acceleration changes faster, so a larger q is
        # needed there — tune via the dashboard.
        Q=100.0,
        R=pd.DataFrame(
            [
                [0.1, 0],
                [0, 0.05],
            ],
            index=pd.RangeIndex(0, 2, name="R"),
        ),
    )

    @staticmethod
    def _ca_process_noise(q: float, dt: float) -> np.ndarray:
        """White-noise-jerk process noise covariance for one CA axis (3x3).

        For a constant-acceleration model driven by white noise on the jerk
        (derivative of acceleration) with power spectral density ``q``, the
        discrete-time process noise covariance over a step ``dt`` is the
        Bar-Shalom constant-acceleration matrix:

            q * [[dt**5/20, dt**4/8,  dt**3/6],
                 [dt**4/8,  dt**3/3,  dt**2/2],
                 [dt**3/6,  dt**2/2,  dt     ]]

        The off-diagonal terms encode the kinematic correlation between the
        position, velocity, and acceleration noise, so the covariance is
        physically consistent (unlike a diagonal Q, which treats the three as
        independent and breaks NEES consistency).
        """
        return q * np.array(
            [
                [dt**5 / 20, dt**4 / 8, dt**3 / 6],
                [dt**4 / 8, dt**3 / 3, dt**2 / 2],
                [dt**3 / 6, dt**2 / 2, dt],
            ]
        )

    def __init__(self, plant: PlantProtocol, sim_params, Q, R):
        dt = sim_params.dt_control

        # Linearized continuous-time input gains: force -> [x_ddot, phi_ddot].
        # Pulled from the plant (not hardcoded) so the filter stays correct
        # when mass / friction / length are changed in the dashboard.
        A_cont, B_cont, _, _ = plant.linear_state_space()
        self._b2 = float(B_cont[1, 0])  # force -> cart acceleration
        self._b4 = float(B_cont[3, 0])  # force -> pendulum angular acceleration

        # State transition: two independent constant-acceleration blocks for
        # [x, x_dot, x_ddot] and [phi, phi_dot, phi_ddot].
        A = np.array(
            [
                [1, dt, 0.5 * dt**2, 0, 0, 0],
                [0, 1, dt, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, dt, 0.5 * dt**2],
                [0, 0, 0, 0, 1, dt],
                [0, 0, 0, 0, 0, 1],
            ]
        )

        # Discrete control input matrix. The force enters at the *acceleration*
        # level (it sets the acceleration via the linearized gain, not the
        # jerk), so over a ZOH step it affects position through the double
        # integrator [½dt², dt]. The acceleration *state* (the residual the CA
        # random walk estimates) is NOT driven by the force (B[accel] = 0):
        # injecting b2*u there would make the acceleration state accumulate the
        # force every step and grow without bound under a constant force. The
        # total acceleration is reconstructed as residual + gain*u in update().
        B = np.array(
            [
                [0.5 * dt**2 * self._b2],
                [dt * self._b2],
                [0],
                [0.5 * dt**2 * self._b4],
                [dt * self._b4],
                [0],
            ]
        )

        C = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
            ]
        )

        # Structured white-noise-jerk process noise: one CA block per axis,
        # built from the scalar jerk density ``Q`` (= ``q``). Both axes share
        # the same intensity for simplicity; the 6x6 matrix is block-diagonal
        # because the CA kinematics decouple x and phi.
        q = float(Q)
        Q_mat = np.zeros((6, 6))
        Q_block = self._ca_process_noise(q, dt)
        Q_mat[:3, :3] = Q_block
        Q_mat[3:, 3:] = Q_block

        observer = KalmanFilter(
            A,
            B,
            C,
            Q_mat,
            R,
        )

        # Initialize the acceleration states from the linearized plant at the
        # initial condition. At startup the control force is zero, so the
        # residual acceleration equals the total acceleration, which is
        # dominated by the gravity restoring torque (e.g. A43*phi). Starting
        # from zero (as a naive CA filter would) leaves the filter to discover
        # a ~14 rad/s^2 acceleration from scratch through tiny position
        # changes, causing a large initial transient and NEES blow-up.
        x0 = np.array(
            [
                sim_params.x_initial,
                sim_params.x_dot_initial,
                sim_params.phi_initial,
                sim_params.phi_dot_initial,
            ]
        )
        x_ddot_init = float(A_cont[1, 1] * x0[1] + A_cont[1, 2] * x0[2])
        phi_ddot_init = float(A_cont[3, 1] * x0[1] + A_cont[3, 2] * x0[2])
        observer.x_last = np.array(
            [
                sim_params.x_initial,
                sim_params.x_dot_initial,
                x_ddot_init,
                sim_params.phi_initial,
                sim_params.phi_dot_initial,
                phi_ddot_init,
            ]
        ).reshape((-1, 1))
        observer.P_last = np.eye(np.size(A, 1)) * sim_params.noise_value**2

        self.observer = observer
        self.plant = plant
        self.estimate_history: list = []
        self.cov_history: list = []
        self.t_history: list = []

    def update(
        self, control_force: float, state: np.ndarray, time: float
    ) -> np.ndarray:
        """Filter one step and record the total-acceleration estimate.

        Delegates to the base class for the predict/correct and the 4D plant
        state extraction, then adds the linearized control-force contribution
        (``b2*u``, ``b4*u``) back into the recorded acceleration states. The
        observer's acceleration states track only the residual (unmodeled)
        acceleration; the total acceleration is ``residual + gain*u``. Recording
        the total lets the NEES diagnostic and the estimate plot compare
        against the plant's true (total) acceleration. The returned 4D state
        and the observer's internal ``x_last`` are unaffected (the controller
        only reads indices 0,1,3,4, and the residual is the correct value for
        the next prediction).
        """
        result = super().update(control_force, state, time)
        u = float(control_force)
        est = self.estimate_history[-1]
        est[2] += self._b2 * u
        est[5] += self._b4 * u
        return result
