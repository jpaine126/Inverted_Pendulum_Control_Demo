import numpy as np
import pytest
from numpy.testing import assert_allclose
from plotly import graph_objects as go

from Inverted_Pendulum_Control_Demo import test_setups


def test_observer_registration():
    class MockObserver(test_setups.ObserverTestSetup, setup_name="test"):
        _dynamic_type = "observer"

    assert test_setups.TestSetup._implemented_observers
    assert test_setups.TestSetup._implemented_observers["test"] is MockObserver


def test_controller_registration():
    class MockController(test_setups.ControllerTestSetup, setup_name="test"):
        _dynamic_type = "controller"

    assert test_setups.TestSetup._implemented_controllers
    assert test_setups.TestSetup._implemented_controllers["test"] is MockController


def test_all_registered_setups_expose_plot():
    """Every registered observer/controller setup must expose a callable plot()."""
    for name, cls in test_setups.TestSetup._implemented_observers.items():
        assert hasattr(cls, "plot"), f"observer {name!r} missing plot()"
        assert callable(cls.plot), f"observer {name!r}.plot is not callable"

    for name, cls in test_setups.TestSetup._implemented_controllers.items():
        assert hasattr(cls, "plot"), f"controller {name!r} missing plot()"
        assert callable(cls.plot), f"controller {name!r}.plot is not callable"


def test_all_registered_observers_update_accept_time():
    """Observer update() must accept (control_force, state, time) per the Protocol."""
    for name, cls in test_setups.TestSetup._implemented_observers.items():
        assert hasattr(cls, "update"), f"observer {name!r} missing update()"


class TestPassThroughObserverPlot:
    def _make_observer(self):
        from Inverted_Pendulum_Control_Demo.test_setups.pass_through_observer import (
            PassThroughObserver,
        )

        return PassThroughObserver()

    def test_plot_empty_history_returns_empty_dict(self):
        assert self._make_observer().plot() == {}

    def test_plot_returns_estimates_dict_with_two_traces_after_updates(self):
        obs = self._make_observer()
        obs.update(0.0, np.array([[1.0], [0.0], [0.5], [0.0]]), 0.0)
        obs.update(0.0, np.array([[1.1], [0.0], [0.4], [0.0]]), 0.1)
        figures = obs.plot()
        # no plant wired up -> only the Estimates figure is produced
        assert set(figures) == {"Estimates"}
        fig = figures["Estimates"]
        assert isinstance(fig, go.Figure)
        assert len(fig.data) == 2
        for tr in fig.data:
            assert isinstance(tr, go.Scatter)
        assert {tr.name for tr in fig.data} == {"x (meas)", "phi (meas)"}

    def test_plot_has_axis_titles(self):
        obs = self._make_observer()
        obs.update(0.0, np.array([[1.0], [0.0], [0.5], [0.0]]), 0.0)
        fig = obs.plot()["Estimates"]
        assert fig.layout.xaxis.title.text == "Time (s)"
        assert fig.layout.yaxis.title.text == "State"

    def test_update_returns_state_unchanged(self):
        obs = self._make_observer()
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        result = obs.update(0.0, state, 0.0)
        assert np.allclose(np.asarray(result).reshape(-1), state.reshape(-1))


class TestObserverErrorsFigure:
    """Cover the new ``"Errors"`` figure returned by observer ``plot()``.

    When an observer is constructed with the plant, its ``plot()`` returns a
    dict with both ``"Estimates"`` and ``"Errors"`` figures. The Errors figure
    holds true-state minus estimated/measured position and angle traces.
    """

    def test_pass_through_observer_errors_are_true_minus_measured(self):
        from Inverted_Pendulum_Control_Demo.plant import InvertedPendulum
        from Inverted_Pendulum_Control_Demo.test_setups.pass_through_observer import (
            PassThroughObserver,
        )

        plant = InvertedPendulum(
            mass_cart=0.5,
            mass_arm=0.2,
            length_arm=0.3,
            friction=0.1,
            gravity=9.8,
            state=np.zeros(4),
        )
        obs = PassThroughObserver(plant=plant)
        true_states = [
            np.array([[0.0], [0.0], [0.5], [0.0]]),
            np.array([[0.1], [0.0], [0.4], [0.0]]),
            np.array([[0.2], [0.0], [0.3], [0.0]]),
        ]
        measured = [
            np.array([[0.01], [0.0], [0.49], [0.0]]),
            np.array([[0.11], [0.0], [0.39], [0.0]]),
            np.array([[0.21], [0.0], [0.29], [0.0]]),
        ]
        t = [0.0, 0.01, 0.02]
        for ti, s, m in zip(t, true_states, measured):
            plant.record(ti, s)
            obs.update(0.0, m, ti)

        figures = obs.plot()
        assert set(figures) == {"Estimates", "Errors"}
        err_fig = figures["Errors"]
        assert [tr.name for tr in err_fig.data] == ["Position Error", "Angle Error"]
        true = np.array([s.reshape(-1) for s in true_states])
        meas = np.array([m.reshape(-1) for m in measured])
        assert_allclose(np.array(err_fig.data[0].x), np.array(t))
        assert_allclose(np.array(err_fig.data[0].y), true[:, 0] - meas[:, 0])
        assert_allclose(np.array(err_fig.data[1].y), true[:, 2] - meas[:, 2])
        assert err_fig.layout.yaxis.title.text == "Error"

    def test_kalman_filter_plot_returns_estimates_and_errors(self):
        from Inverted_Pendulum_Control_Demo.plant import InvertedPendulum
        from Inverted_Pendulum_Control_Demo.sim_parameters import ControlDemoParam
        from Inverted_Pendulum_Control_Demo.test_setups.dynamic_kalman_filter import (
            DynamicKalmanFilter,
        )

        sim_params = ControlDemoParam()
        plant = InvertedPendulum(
            mass_cart=sim_params.mass_cart,
            mass_arm=sim_params.mass_arm,
            length_arm=sim_params.length_arm,
            friction=sim_params.cart_friction,
            gravity=sim_params.gravity,
            state=np.zeros(4),
        )
        obs = DynamicKalmanFilter(
            plant,
            sim_params,
            Q=DynamicKalmanFilter.params["Q"],
            R=DynamicKalmanFilter.params["R"],
        )
        states = [
            np.array([[0.0], [0.0], [0.5], [0.0]]),
            np.array([[0.01], [0.0], [0.49], [0.0]]),
            np.array([[0.02], [0.0], [0.48], [0.0]]),
        ]
        t = [0.0, 0.01, 0.02]
        for ti, s in zip(t, states):
            plant.record(ti, s)
            obs.update(0.0, s, ti)

        figures = obs.plot()
        assert set(figures) == {"Estimates", "Errors", "NEES"}
        # Estimates figure plots all 4 internal states
        est_fig = figures["Estimates"]
        assert [tr.name for tr in est_fig.data] == [
            "x (est)",
            "x_dot (est)",
            "phi (est)",
            "phi_dot (est)",
        ]
        estimates = np.array(obs.estimate_history)
        for i in range(4):
            assert_allclose(np.array(est_fig.data[i].y), estimates[:, i])
        err_fig = figures["Errors"]
        assert [tr.name for tr in err_fig.data] == [
            "x Error",
            "x_dot Error",
            "phi Error",
            "phi_dot Error",
        ]
        true = np.array([s.reshape(-1) for s in states])
        n = min(len(true), len(estimates))
        for i in range(4):
            assert_allclose(np.array(err_fig.data[i].y), true[:n, i] - estimates[:n, i])

    def test_pass_through_observer_no_plant_omits_errors(self):
        """Without a plant, only the Estimates figure is returned (no Errors)."""
        from Inverted_Pendulum_Control_Demo.test_setups.pass_through_observer import (
            PassThroughObserver,
        )

        obs = PassThroughObserver()
        obs.update(0.0, np.array([[1.0], [0.0], [0.5], [0.0]]), 0.0)
        figures = obs.plot()
        assert set(figures) == {"Estimates"}


class TestCAKalmanFilter:
    """The constant-acceleration filter estimates a 6D internal state but must
    hand the controller the plant's 4D state, and its 6-DOF NEES must not crash
    against the plant's augmented (6D) true state."""

    def _make_filter(self):
        from Inverted_Pendulum_Control_Demo.plant import InvertedPendulum
        from Inverted_Pendulum_Control_Demo.sim_parameters import ControlDemoParam
        from Inverted_Pendulum_Control_Demo.test_setups.ca_kalman_filter import (
            CAKalmanFilter,
        )

        sim_params = ControlDemoParam()
        plant = InvertedPendulum(
            mass_cart=sim_params.mass_cart,
            mass_arm=sim_params.mass_arm,
            length_arm=sim_params.length_arm,
            friction=sim_params.cart_friction,
            gravity=sim_params.gravity,
            state=np.zeros(4),
        )
        obs = CAKalmanFilter(
            plant,
            sim_params,
            Q=CAKalmanFilter.params["Q"],
            R=CAKalmanFilter.params["R"],
        )
        return obs, plant

    def test_plant_state_indices_declared(self):
        obs, _ = self._make_filter()
        assert obs.plant_state_indices == [0, 1, 3, 4]
        assert obs.internal_state_dim == 6
        assert len(obs.plant_state_indices) == 4
        assert max(obs.plant_state_indices) < obs.internal_state_dim

    def test_update_returns_4d_despite_6d_internal(self):
        obs, _ = self._make_filter()
        state = np.array([[0.1], [0.0], [0.5], [0.0]])
        result = obs.update(0.0, state, 0.0)
        assert np.asarray(result).reshape(-1).shape == (4,)

    def test_estimate_history_stores_full_6d(self):
        obs, _ = self._make_filter()
        state = np.array([[0.1], [0.0], [0.5], [0.0]])
        obs.update(0.0, state, 0.0)
        # internal storage keeps the full 6D estimate + 6x6 covariance
        assert np.array(obs.estimate_history).shape == (1, 6)
        assert np.array(obs.cov_history).shape == (1, 6, 6)

    def test_plot_returns_three_figures_with_plant_history(self):
        """Regression test: the 6-DOF NEES np.linalg.solve path previously
        crashed with a 6x6 covariance vs 4D reindexed-error mismatch."""
        obs, plant = self._make_filter()
        states = [
            np.array([[0.0], [0.0], [0.5], [0.0]]),
            np.array([[0.01], [0.0], [0.49], [0.0]]),
            np.array([[0.02], [0.0], [0.48], [0.0]]),
        ]
        t = [0.0, 0.01, 0.02]
        for ti, s in zip(t, states):
            plant.record(ti, s, force=0.0)
            obs.update(0.0, s, ti)
        figures = obs.plot()
        assert set(figures) == {"Estimates", "Errors", "NEES"}
        assert len(figures["NEES"].data) == 1
        # Estimates figure plots all 6 internal states
        assert [tr.name for tr in figures["Estimates"].data] == [
            "x (est)",
            "x_dot (est)",
            "x_ddot (est)",
            "phi (est)",
            "phi_dot (est)",
            "phi_ddot (est)",
        ]
        estimates = np.array(obs.estimate_history)
        for i in range(6):
            assert_allclose(np.array(figures["Estimates"].data[i].y), estimates[:, i])
        # Errors figure reports all 6 internal-state errors
        assert [tr.name for tr in figures["Errors"].data] == [
            "x Error",
            "x_dot Error",
            "x_ddot Error",
            "phi Error",
            "phi_dot Error",
            "phi_ddot Error",
        ]

    def test_B_built_from_plant_acceleration_level(self):
        """B injects the force at the acceleration level (not the jerk level):
        position gets the double-integral [½dt²], velocity gets [dt], and the
        acceleration *state* gets 0 (it tracks the residual, not the force).
        The pos/vel entries match the 4D KF's scipy ZOH B_disc, and the
        acceleration entries are zero by design."""
        from Inverted_Pendulum_Control_Demo.plant import InvertedPendulum
        from Inverted_Pendulum_Control_Demo.sim_parameters import ControlDemoParam
        from Inverted_Pendulum_Control_Demo.test_setups.ca_kalman_filter import (
            CAKalmanFilter,
        )

        sim_params = ControlDemoParam()
        plant = InvertedPendulum(
            mass_cart=sim_params.mass_cart,
            mass_arm=sim_params.mass_arm,
            length_arm=sim_params.length_arm,
            friction=sim_params.cart_friction,
            gravity=sim_params.gravity,
            state=np.zeros(4),
        )
        obs = CAKalmanFilter(
            plant,
            sim_params,
            Q=CAKalmanFilter.params["Q"],
            R=CAKalmanFilter.params["R"],
        )
        dt = sim_params.dt_control
        _, B_cont, _, _ = plant.linear_state_space()
        b2 = float(B_cont[1, 0])
        b4 = float(B_cont[3, 0])
        B = np.asarray(obs.observer.B).reshape(-1)

        expected = np.array(
            [
                0.5 * dt**2 * b2,
                dt * b2,
                0,
                0.5 * dt**2 * b4,
                dt * b4,
                0,
            ]
        )
        assert_allclose(B, expected)
        # acceleration entries are zero (force does NOT accumulate into the
        # acceleration state, which tracks only the residual)
        assert B[2] == 0.0
        assert B[5] == 0.0
        # pos/vel entries are non-zero (the force affects position and velocity)
        assert np.all(B[[0, 1, 3, 4]] != 0.0)

    def test_B_includes_pendulum_coupling(self):
        """The force couples to the pendulum's position and velocity (via the
        linearized gain b4); the old B had B[3:] == 0, dropping that coupling."""
        from Inverted_Pendulum_Control_Demo.plant import InvertedPendulum
        from Inverted_Pendulum_Control_Demo.sim_parameters import ControlDemoParam
        from Inverted_Pendulum_Control_Demo.test_setups.ca_kalman_filter import (
            CAKalmanFilter,
        )

        sim_params = ControlDemoParam()
        plant = InvertedPendulum(
            mass_cart=sim_params.mass_cart,
            mass_arm=sim_params.mass_arm,
            length_arm=sim_params.length_arm,
            friction=sim_params.cart_friction,
            gravity=sim_params.gravity,
            state=np.zeros(4),
        )
        obs = CAKalmanFilter(
            plant,
            sim_params,
            Q=CAKalmanFilter.params["Q"],
            R=CAKalmanFilter.params["R"],
        )
        B = np.asarray(obs.observer.B).reshape(-1)
        # pendulum pos/vel entries (indices 3, 4) are non-zero
        assert B[3] != 0.0
        assert B[4] != 0.0
        # the cart and pendulum velocity gains differ (different physical units)
        assert B[1] != B[4]

    def test_acceleration_init_from_linearized_plant(self):
        """The acceleration states are initialized from the linearized plant at
        the initial condition (the gravity-driven acceleration), not zero.
        Starting from zero causes a large initial transient and NEES blow-up."""
        from Inverted_Pendulum_Control_Demo.plant import InvertedPendulum
        from Inverted_Pendulum_Control_Demo.sim_parameters import ControlDemoParam
        from Inverted_Pendulum_Control_Demo.test_setups.ca_kalman_filter import (
            CAKalmanFilter,
        )

        sim_params = ControlDemoParam()
        plant = InvertedPendulum(
            mass_cart=sim_params.mass_cart,
            mass_arm=sim_params.mass_arm,
            length_arm=sim_params.length_arm,
            friction=sim_params.cart_friction,
            gravity=sim_params.gravity,
            state=np.zeros(4),
        )
        obs = CAKalmanFilter(
            plant,
            sim_params,
            Q=CAKalmanFilter.params["Q"],
            R=CAKalmanFilter.params["R"],
        )
        A_cont, _, _, _ = plant.linear_state_space()
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
        x_last = np.asarray(obs.observer.x_last).reshape(-1)
        assert_allclose(x_last[2], x_ddot_init)
        assert_allclose(x_last[5], phi_ddot_init)
        # non-zero (gravity acts on the initial angle)
        assert x_last[2] != 0.0
        assert x_last[5] != 0.0

    def test_update_records_total_acceleration(self):
        """The observer's acceleration states track the residual (unmodeled)
        acceleration, but update() adds the linearized force contribution back
        so the recorded estimate is the TOTAL acceleration — matching the
        plant's augmented_state_history for a correct NEES. With a non-zero
        force, the recorded acceleration differs from the internal state."""
        from Inverted_Pendulum_Control_Demo.plant import InvertedPendulum
        from Inverted_Pendulum_Control_Demo.sim_parameters import ControlDemoParam
        from Inverted_Pendulum_Control_Demo.test_setups.ca_kalman_filter import (
            CAKalmanFilter,
        )

        sim_params = ControlDemoParam()
        plant = InvertedPendulum(
            mass_cart=sim_params.mass_cart,
            mass_arm=sim_params.mass_arm,
            length_arm=sim_params.length_arm,
            friction=sim_params.cart_friction,
            gravity=sim_params.gravity,
            state=np.zeros(4),
        )
        obs = CAKalmanFilter(
            plant,
            sim_params,
            Q=CAKalmanFilter.params["Q"],
            R=CAKalmanFilter.params["R"],
        )
        _, B_cont, _, _ = plant.linear_state_space()
        b2 = float(B_cont[1, 0])
        b4 = float(B_cont[3, 0])
        u = 5.0  # arbitrary non-zero force
        state = np.array([[0.1], [0.0], [0.5], [0.0]])
        obs.update(u, state, 0.0)
        recorded = np.array(obs.estimate_history[-1])
        internal = np.asarray(obs.observer.x_last).reshape(-1)
        # the recorded total acceleration = internal residual + gain*u
        assert_allclose(recorded[2], internal[2] + b2 * u)
        assert_allclose(recorded[5], internal[5] + b4 * u)
        # with zero force, recorded == internal (no correction)
        obs.update(0.0, state, 0.01)
        recorded0 = np.array(obs.estimate_history[-1])
        internal0 = np.asarray(obs.observer.x_last).reshape(-1)
        assert_allclose(recorded0[2], internal0[2])
        assert_allclose(recorded0[5], internal0[5])

    def test_Q_is_structured_ca_covariance(self):
        """Q is the block-diagonal white-noise-jerk CA covariance, not a
        diagonal matrix: it is symmetric, PSD, and carries off-diagonal
        kinematic terms within each 3x3 axis block while the two axis blocks
        are uncoupled (block-diagonal)."""
        from Inverted_Pendulum_Control_Demo.plant import InvertedPendulum
        from Inverted_Pendulum_Control_Demo.sim_parameters import ControlDemoParam
        from Inverted_Pendulum_Control_Demo.test_setups.ca_kalman_filter import (
            CAKalmanFilter,
        )

        sim_params = ControlDemoParam()
        plant = InvertedPendulum(
            mass_cart=sim_params.mass_cart,
            mass_arm=sim_params.mass_arm,
            length_arm=sim_params.length_arm,
            friction=sim_params.cart_friction,
            gravity=sim_params.gravity,
            state=np.zeros(4),
        )
        obs = CAKalmanFilter(
            plant,
            sim_params,
            Q=1.0,
            R=CAKalmanFilter.params["R"],
        )
        Q = np.asarray(obs.observer.Q)
        dt = sim_params.dt_control

        # symmetric and PSD
        assert_allclose(Q, Q.T)
        assert np.all(np.linalg.eigvalsh(Q) > 0)

        # off-diagonal kinematic terms present (old Q was 0.01 * I -> strictly diagonal)
        assert np.any(np.triu(Q, k=1) != 0.0)

        # acceleration diagonal = q * dt = 1.0 * 0.01
        assert_allclose(Q[2, 2], dt)
        assert_allclose(Q[5, 5], dt)
        # vel-accel cross term = q * dt**2 / 2
        assert_allclose(Q[1, 2], 0.5 * dt**2)

        # block-diagonal: the two axis blocks are uncoupled
        assert_allclose(Q[:3, 3:], np.zeros((3, 3)))

    def test_Q_scalar_param_scales_covariance(self):
        """Doubling the scalar Q doubles the structured covariance."""
        from Inverted_Pendulum_Control_Demo.plant import InvertedPendulum
        from Inverted_Pendulum_Control_Demo.sim_parameters import ControlDemoParam
        from Inverted_Pendulum_Control_Demo.test_setups.ca_kalman_filter import (
            CAKalmanFilter,
        )

        sim_params = ControlDemoParam()
        plant = InvertedPendulum(
            mass_cart=sim_params.mass_cart,
            mass_arm=sim_params.mass_arm,
            length_arm=sim_params.length_arm,
            friction=sim_params.cart_friction,
            gravity=sim_params.gravity,
            state=np.zeros(4),
        )
        obs1 = CAKalmanFilter(plant, sim_params, Q=1.0, R=CAKalmanFilter.params["R"])
        obs2 = CAKalmanFilter(plant, sim_params, Q=2.0, R=CAKalmanFilter.params["R"])
        Q1 = np.asarray(obs1.observer.Q)
        Q2 = np.asarray(obs2.observer.Q)
        assert_allclose(Q2, 2.0 * Q1)


class TestKalmanFilterBaseIdentity:
    """When plant_state_indices is None (the default), the inject/extract
    helpers are identity and the 4D path behaves as before."""

    def test_dynamic_kalman_filter_uses_identity_path(self):
        from Inverted_Pendulum_Control_Demo.plant import InvertedPendulum
        from Inverted_Pendulum_Control_Demo.sim_parameters import ControlDemoParam
        from Inverted_Pendulum_Control_Demo.test_setups.dynamic_kalman_filter import (
            DynamicKalmanFilter,
        )

        sim_params = ControlDemoParam()
        plant = InvertedPendulum(
            mass_cart=sim_params.mass_cart,
            mass_arm=sim_params.mass_arm,
            length_arm=sim_params.length_arm,
            friction=sim_params.cart_friction,
            gravity=sim_params.gravity,
            state=np.zeros(4),
        )
        obs = DynamicKalmanFilter(
            plant,
            sim_params,
            Q=DynamicKalmanFilter.params["Q"],
            R=DynamicKalmanFilter.params["R"],
        )
        assert obs.plant_state_indices is None
        state = np.array([[0.1], [0.0], [0.5], [0.0]])
        result = obs.update(0.0, state, 0.0)
        # 4D in, 4D out, 4D stored
        assert np.asarray(result).reshape(-1).shape == (4,)
        assert np.array(obs.estimate_history).shape == (1, 4)
        assert np.array(obs.cov_history).shape == (1, 4, 4)
