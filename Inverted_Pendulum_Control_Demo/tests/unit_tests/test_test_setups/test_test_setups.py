import numpy as np
import plotly.graph_objects as go
import pytest

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

    def test_plot_empty_history_returns_empty_list(self):
        assert self._make_observer().plot() == []

    def test_plot_returns_two_traces_after_updates(self):
        obs = self._make_observer()
        obs.update(0.0, np.array([[1.0], [0.0], [0.5], [0.0]]), 0.0)
        obs.update(0.0, np.array([[1.1], [0.0], [0.4], [0.0]]), 0.1)
        traces = obs.plot()
        assert len(traces) == 2
        for tr in traces:
            assert isinstance(tr, go.Scatter)
        assert {tr.name for tr in traces} == {"x (meas)", "phi (meas)"}

    def test_update_returns_state_unchanged(self):
        obs = self._make_observer()
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        result = obs.update(0.0, state, 0.0)
        assert np.allclose(np.asarray(result).reshape(-1), state.reshape(-1))
