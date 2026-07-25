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
        from Inverted_Pendulum_Control_Demo.test_setups.basic_kalman_filter import (
            BasicKalmanFilter,
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
        obs = BasicKalmanFilter(
            plant,
            sim_params,
            Q=BasicKalmanFilter.params["Q"],
            R=BasicKalmanFilter.params["R"],
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
        assert set(figures) == {"Estimates", "Errors"}
        err_fig = figures["Errors"]
        assert [tr.name for tr in err_fig.data] == ["Position Error", "Angle Error"]
        true = np.array([s.reshape(-1) for s in states])
        estimates = np.array(obs.estimate_history)
        n = min(len(true), len(estimates))
        assert_allclose(np.array(err_fig.data[0].y), true[:n, 0] - estimates[:n, 0])
        assert_allclose(np.array(err_fig.data[1].y), true[:n, 2] - estimates[:n, 2])

    def test_pass_through_observer_no_plant_omits_errors(self):
        """Without a plant, only the Estimates figure is returned (no Errors)."""
        from Inverted_Pendulum_Control_Demo.test_setups.pass_through_observer import (
            PassThroughObserver,
        )

        obs = PassThroughObserver()
        obs.update(0.0, np.array([[1.0], [0.0], [0.5], [0.0]]), 0.0)
        figures = obs.plot()
        assert set(figures) == {"Estimates"}
