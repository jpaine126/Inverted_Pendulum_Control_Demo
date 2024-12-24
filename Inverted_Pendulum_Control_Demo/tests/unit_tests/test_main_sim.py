from unittest.mock import MagicMock

import numpy as np
import pytest
from numpy.testing import assert_allclose

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
