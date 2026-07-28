from unittest.mock import MagicMock

import numpy as np
import pytest
from numpy.testing import assert_allclose

from Inverted_Pendulum_Control_Demo import main_sim
from Inverted_Pendulum_Control_Demo.main_sim import (
    MainSim,
    get_noise,
    value_or_full_like,
)


class TestValueOrFullLike:
    def test_returns_value_when_provided(self):
        array_like = np.zeros(4)
        value = np.array([1.0, 2.0, 3.0, 4.0])
        result = value_or_full_like(value, array_like, 0.0)
        assert_allclose(result, [1.0, 2.0, 3.0, 4.0])

    def test_returns_full_like_when_none(self):
        array_like = np.array([1.0, 2.0, 3.0, 4.0])
        result = value_or_full_like(None, array_like, 0.0)
        assert_allclose(result, [0.0, 0.0, 0.0, 0.0])
        assert result.shape == array_like.shape

    def test_uses_fill_value(self):
        array_like = np.array([1.0, 2.0])
        result = value_or_full_like(None, array_like, 5.0)
        assert_allclose(result, [5.0, 5.0])

    def test_preserves_dtype(self):
        array_like = np.array([1.0, 2.0], dtype=np.float64)
        result = value_or_full_like(None, array_like, 5.0)
        assert result.dtype == np.float64

    def test_works_with_2d_array_like(self):
        array_like = np.zeros((4, 1))
        result = value_or_full_like(None, array_like, 0.0)
        assert result.shape == (4, 1)
        assert_allclose(result, np.zeros((4, 1)))


class TestGetNoise:
    def test_default_shape(self):
        noise = get_noise(0.1)
        assert noise.shape == (4, 1)

    def test_custom_shape(self):
        noise = get_noise(0.1, size=(2, 3))
        assert noise.shape == (2, 3)

    def test_magnitude_bounded(self):
        # Values should always be in [-magnitude, +magnitude]
        for _ in range(20):
            noise = get_noise(0.5)
            assert np.all(np.abs(noise) <= 0.5)

    def test_zero_magnitude_returns_zeros(self):
        noise = get_noise(0.0)
        assert_allclose(noise, np.zeros((4, 1)))


def _make_sim(**kwargs):
    """Build a MainSim with mock plant/observer/controller and sensible defaults."""
    controller = MagicMock()
    observer = MagicMock()
    observer.update.return_value = np.array([[1.0], [2.0], [3.0], [4.0]])
    controller.update.return_value = 7.5
    plant = MagicMock()
    defaults = dict(
        controller=controller,
        observer=observer,
        plant=plant,
        initial_conditions=np.zeros((4, 1)),
        measurement_noise_value=0.001,
    )
    defaults.update(kwargs)
    return MainSim(**defaults), controller, observer


class TestMainSimInit:
    def test_stores_dependencies(self):
        controller = MagicMock()
        observer = MagicMock()
        plant = MagicMock()
        initial = np.zeros((4, 1))
        sim = MainSim(
            controller=controller,
            observer=observer,
            plant=plant,
            initial_conditions=initial,
            measurement_noise_value=0.001,
        )
        assert sim.controller is controller
        assert sim.observer is observer
        assert sim.plant is plant
        assert_allclose(sim.initial_conditions, initial)
        assert_allclose(sim.state, initial)

    def test_default_dt_and_t_final(self):
        sim, _, _ = _make_sim()
        assert sim.dt_control == 0.02
        # default t_final = 10.0 → steps = ceil(10/0.02) = 500
        assert sim.steps == 500
        assert sim.t_control.shape == (500,)

    def test_custom_dt_and_t_final(self):
        sim, _, _ = _make_sim(dt_control=0.01, t_final=1.0)
        assert sim.t_control.shape == (100,)
        assert sim.steps == 100
        assert_allclose(sim.t_control[0], 0.0)
        assert_allclose(sim.t_control[-1], 0.99)

    def test_default_noise_off(self):
        sim, _, _ = _make_sim()
        assert sim.measure_noise is False
        assert sim.measurement_noise_value == 0.001

    def test_default_sensor_arrays_are_zeros(self):
        initial = np.array([[1.0], [2.0], [3.0], [4.0]])
        sim, _, _ = _make_sim(initial_conditions=initial)
        assert_allclose(sim.sensor_discretize, np.zeros_like(initial))
        assert_allclose(sim.sensor_discretize_offset, np.zeros_like(initial))
        assert_allclose(sim.sensor_bias, np.zeros_like(initial))

    def test_provided_sensor_arrays_used_unchanged(self):
        discretize = np.array([[0.1], [0.2], [0.3], [0.4]])
        offset = np.array([[0.01], [0.02], [0.03], [0.04]])
        bias = np.array([[0.5], [0.6], [0.7], [0.8]])
        sim, _, _ = _make_sim(
            sensor_discretize=discretize,
            sensor_discretize_offset=offset,
            sensor_bias=bias,
        )
        assert_allclose(sim.sensor_discretize, discretize)
        assert_allclose(sim.sensor_discretize_offset, offset)
        assert_allclose(sim.sensor_bias, bias)

    def test_histories_initialize_empty(self):
        sim, _, _ = _make_sim()
        assert sim.state_history == [[], [], [], []]
        assert sim.adjusted_state_history == [[], [], [], []]
        assert sim.measurement_history == [[], [], [], []]
        assert sim.control_force_history == []


class TestMainSimRecord:
    def test_record_grows_state_history(self):
        sim, _, _ = _make_sim()
        real_state = np.array([[1.0], [2.0], [3.0], [4.0]])
        adjusted = np.array([[1.1], [2.1], [3.1], [4.1]])
        measurement = np.array([[1.0], [2.0], [3.0], [4.0]])
        sim.record(real_state, adjusted, measurement, 5.0)
        assert sim.state_history.shape == (4, 1)
        assert_allclose(sim.state_history, real_state)
        assert_allclose(sim.adjusted_state_history, adjusted)
        assert_allclose(sim.measurement_history, measurement)
        assert np.allclose(sim.control_force_history, [5.0])

    def test_record_appends_second_step(self):
        sim, _, _ = _make_sim()
        first = np.array([[1.0], [2.0], [3.0], [4.0]])
        second = np.array([[5.0], [6.0], [7.0], [8.0]])
        sim.record(first, first, first, 1.0)
        sim.record(second, second, second, 2.0)
        assert sim.state_history.shape == (4, 2)
        assert_allclose(sim.state_history[:, 0], first.ravel())
        assert_allclose(sim.state_history[:, 1], second.ravel())
        assert np.allclose(sim.control_force_history, [1.0, 2.0])


@pytest.mark.filterwarnings("ignore::RuntimeWarning")
class TestMainSimControlStep:
    def test_no_noise_passes_state_to_observer(self):
        sim, controller, observer = _make_sim(measurement_noise=False)
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        sim.control_step(state, 0.0, 0.0)
        # No noise, no discretize (bins=0 pass-through), no bias → final_data == state
        call_args = observer.update.call_args
        assert call_args[0][0] == 0.0
        assert_allclose(call_args[0][1], state)

    def test_observer_output_passed_to_controller(self):
        sim, controller, observer = _make_sim(measurement_noise=False)
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        sim.control_step(state, 0.0, 1.5)
        # controller.update(measurement=observer's return, time)
        call_args = controller.update.call_args
        assert call_args[0][1] == 1.5
        assert_allclose(call_args[0][0], observer.update.return_value)

    def test_sensor_bias_added(self):
        sim, controller, observer = _make_sim(measurement_noise=False)
        sim.sensor_bias = np.array([[0.1], [0.2], [0.3], [0.4]])
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        sim.control_step(state, 0.0, 0.0)
        final_data = observer.update.call_args[0][1]
        expected = state + np.array([[0.1], [0.2], [0.3], [0.4]])
        assert_allclose(final_data, expected)

    def test_discretize_rounds_to_bin(self):
        sim, controller, observer = _make_sim(measurement_noise=False)
        sim.sensor_discretize = np.array([[0.5], [0.5], [0.5], [0.5]])
        state = np.array([[1.3], [-0.7], [0.0], [2.6]])
        sim.control_step(state, 0.0, 0.0)
        final_data = observer.update.call_args[0][1]
        # 1.3 → 1.0, -0.7 → -0.5, 0.0 → 0.0, 2.6 → 2.5
        expected = np.array([[1.0], [-0.5], [0.0], [2.5]])
        assert_allclose(final_data, expected)

    def test_zero_bin_skips_discretize_for_that_dim(self):
        sim, controller, observer = _make_sim(measurement_noise=False)
        sim.sensor_discretize = np.array([[0.5], [0.0], [0.5], [0.0]])
        state = np.array([[1.3], [0.123], [2.6], [0.456]])
        sim.control_step(state, 0.0, 0.0)
        final_data = observer.update.call_args[0][1]
        # Discretized dims: 1.3→1.0, 2.6→2.5; pass-through dims: 0.123, 0.456
        expected = np.array([[1.0], [0.123], [2.5], [0.456]])
        assert_allclose(final_data, expected)

    def test_discretize_offset_applied(self):
        sim, controller, observer = _make_sim(measurement_noise=False)
        sim.sensor_discretize = np.array([[0.5], [0.5], [0.5], [0.5]])
        sim.sensor_discretize_offset = np.array([[0.01], [0.02], [0.03], [0.04]])
        state = np.array([[1.3], [-0.7], [0.0], [2.6]])
        sim.control_step(state, 0.0, 0.0)
        final_data = observer.update.call_args[0][1]
        expected = np.array([[1.0 + 0.01], [-0.5 + 0.02], [0.0 + 0.03], [2.5 + 0.04]])
        assert_allclose(final_data, expected)

    def test_record_called_within_control_step(self):
        sim, controller, observer = _make_sim(measurement_noise=False)
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        sim.control_step(state, 0.0, 0.0)
        assert sim.state_history.shape == (4, 1)
        assert_allclose(sim.state_history, state)
        assert np.allclose(sim.control_force_history, [7.5])

    def test_noise_adds_to_state_when_enabled(self):
        sim, controller, observer = _make_sim(
            measurement_noise=True, measurement_noise_value=0.0
        )
        # With magnitude 0, noise is zero, so state passes through unchanged
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        sim.control_step(state, 0.0, 0.0)
        final_data = observer.update.call_args[0][1]
        assert_allclose(final_data, state)


@pytest.mark.filterwarnings("ignore::RuntimeWarning")
class TestMainSimPlantRecord:
    """Tests that control_step records state history on the plant."""

    def test_plant_record_called_with_time_and_state(self):
        sim, _, _ = _make_sim(measurement_noise=False)
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        sim.control_step(state, 0.0, 7.5)
        call_args = sim.plant.record.call_args
        # plant.record(time, state, force)
        assert call_args[0][0] == 7.5
        assert_allclose(call_args[0][1], state)

    def test_plant_record_receives_controller_force(self):
        sim, _, _ = _make_sim(measurement_noise=False)
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        sim.control_step(state, 0.0, 7.5)
        call_args = sim.plant.record.call_args
        # the force passed is the controller's output (return_value == 7.5)
        assert call_args[0][2] == 7.5

    def test_plant_record_called_each_control_step(self):
        sim, _, _ = _make_sim(measurement_noise=False)
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        sim.control_step(state, 0.0, 1.0)
        sim.control_step(state, 0.0, 2.0)
        sim.control_step(state, 0.0, 3.0)
        assert sim.plant.record.call_count == 3

    def test_plant_record_not_called_outside_control_step(self):
        sim, _, _ = _make_sim()
        # Without calling control_step, plant.record should not have been called
        sim.plant.record.assert_not_called()


@pytest.mark.filterwarnings("ignore::RuntimeWarning")
class TestMainSimObserverTimeArg:
    """Tests that observer.update receives the time argument."""

    def test_observer_update_receives_time(self):
        sim, _, observer = _make_sim(measurement_noise=False)
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        sim.control_step(state, 0.0, 4.2)
        call_args = observer.update.call_args
        # observer.update(control_force, final_data, time)
        assert call_args[0][0] == 0.0
        assert_allclose(call_args[0][1], state)
        assert call_args[0][2] == 4.2

    def test_observer_update_called_with_three_positional_args(self):
        sim, _, observer = _make_sim(measurement_noise=False)
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        sim.control_step(state, 1.0, 0.5)
        call_args = observer.update.call_args
        assert len(call_args[0]) == 3


def _fake_solve_ivp(states):
    """Build a callable replacing ``solve_ivp`` that yields ``states`` in order.

    Each element of ``states`` is a length-4 sequence used as the propagated
    state's final column. Inputs are ignored; the returned object's ``.y`` is
    shaped ``(4, 1)`` so ``a.y[:, -1]`` reproduces the element (matching what
    ``run_sim`` does with ``np.atleast_2d(a.y[:, -1]).T``).
    """
    queue = [np.asarray(s, dtype=float).reshape((4, 1)) for s in states]
    call = {"n": 0}

    def _ivp(*args, **kwargs):
        idx = min(call["n"], len(queue) - 1)
        call["n"] += 1

        class _Result:
            pass

        r = _Result()
        r.y = queue[idx]
        return r

    return _ivp


def _make_runnable_sim(**kwargs):
    """Build a MainSim with mocks and a real (4,1) plant.state for run_sim."""
    sim, controller, observer = _make_sim(
        measurement_noise=False, t_final=0.1, dt_control=0.02, **kwargs
    )
    sim.plant.state = np.zeros((4, 1))
    return sim, controller, observer


@pytest.mark.filterwarnings("ignore::RuntimeWarning")
class TestMainSimRunSimEarlyStop:
    """Tests for run_sim's configurable blow-up early stop."""

    def test_default_threshold_value(self):
        sim, _, _ = _make_runnable_sim()
        assert sim.state_blowup_threshold == main_sim.STATE_BLOWUP_THRESHOLD
        assert sim.state_blowup_threshold == 100.0

    def test_custom_threshold_stored(self):
        sim, _, _ = _make_runnable_sim(state_blowup_threshold=42.0)
        assert sim.state_blowup_threshold == 42.0

    def test_runs_all_steps_when_bounded(self, monkeypatch):
        sim, _, _ = _make_runnable_sim()
        # ceil(0.1 / 0.02) = 5 steps; keep state at zero so it never blows up
        monkeypatch.setattr(
            main_sim, "solve_ivp", _fake_solve_ivp([[0, 0, 0, 0]] * sim.steps)
        )
        sim.run_sim()
        assert sim.state_history.shape == (4, sim.steps)
        assert sim.control_force_history.shape == (sim.steps,)
        assert sim.plant.record.call_count == sim.steps
        assert sim.observer.update.call_count == sim.steps
        assert sim.controller.update.call_count == sim.steps

    def test_stops_early_when_state_exceeds_default_threshold(self, monkeypatch):
        sim, _, _ = _make_runnable_sim()
        # step 0 -> 0 (ok), step 1 -> 200 (exceeds default 100) -> break
        monkeypatch.setattr(
            main_sim, "solve_ivp", _fake_solve_ivp([[0, 0, 0, 0], [200, 0, 0, 0]])
        )
        sim.run_sim()
        # 2 control_step calls recorded before the propagated state blew up;
        # the blown-up state itself is never recorded.
        assert sim.state_history.shape == (4, 2)
        assert sim.control_force_history.shape == (2,)
        assert sim.plant.record.call_count == 2
        assert sim.observer.update.call_count == 2
        assert sim.controller.update.call_count == 2

    def test_plotting_dims_match_after_early_stop(self, monkeypatch):
        sim, _, _ = _make_runnable_sim()
        monkeypatch.setattr(
            main_sim, "solve_ivp", _fake_solve_ivp([[0, 0, 0, 0], [200, 0, 0, 0]])
        )
        sim.run_sim()
        n = sim.state_history.shape[1]
        # Every history a plot consumes (MainSim, plant, observer, controller)
        # must hold the same number of samples so trace lengths line up.
        assert sim.state_history.shape == (4, n)
        assert sim.adjusted_state_history.shape == (4, n)
        assert sim.measurement_history.shape == (4, n)
        assert sim.control_force_history.shape == (n,)
        assert sim.plant.record.call_count == n
        assert sim.observer.update.call_count == n
        assert sim.controller.update.call_count == n
        # the blown-up propagated state was not recorded into the plant state
        assert np.all(np.abs(sim.state_history) <= sim.state_blowup_threshold)

    def test_custom_threshold_triggers_earlier_break(self, monkeypatch):
        # 75 exceeds a custom threshold of 50 but not the default of 100
        sim, _, _ = _make_runnable_sim(state_blowup_threshold=50.0)
        monkeypatch.setattr(
            main_sim, "solve_ivp", _fake_solve_ivp([[0, 0, 0, 0], [75, 0, 0, 0]])
        )
        sim.run_sim()
        assert sim.state_history.shape == (4, 2)

    def test_default_threshold_does_not_break_below_100(self, monkeypatch):
        sim, _, _ = _make_runnable_sim()
        monkeypatch.setattr(
            main_sim, "solve_ivp", _fake_solve_ivp([[75, 0, 0, 0]] * sim.steps)
        )
        sim.run_sim()
        assert sim.state_history.shape == (4, sim.steps)

    def test_stops_on_nonfinite_state(self, monkeypatch):
        sim, _, _ = _make_runnable_sim()
        monkeypatch.setattr(
            main_sim, "solve_ivp", _fake_solve_ivp([[0, 0, 0, 0], [np.nan, 0, 0, 0]])
        )
        sim.run_sim()
        # NaN in the propagated state is treated as a blow-up and stops the sim
        assert sim.state_history.shape == (4, 2)

    def test_stops_on_infinite_state(self, monkeypatch):
        sim, _, _ = _make_runnable_sim()
        monkeypatch.setattr(
            main_sim, "solve_ivp", _fake_solve_ivp([[0, 0, 0, 0], [np.inf, 0, 0, 0]])
        )
        sim.run_sim()
        assert sim.state_history.shape == (4, 2)
