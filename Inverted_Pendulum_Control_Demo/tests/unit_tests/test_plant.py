import math

import numpy as np
import plotly.graph_objects as go
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


@pytest.fixture
def plant_with_history(plant):
    """Plant with a few recorded samples for plot/animate tests."""
    samples = [
        (0.0, np.array([[0.0], [0.0], [0.0], [0.0]])),
        (0.1, np.array([[0.01], [0.1], [0.2], [0.5]])),
        (0.2, np.array([[0.04], [0.2], [0.1], [0.3]])),
        (0.3, np.array([[0.09], [0.3], [0.0], [0.0]])),
    ]
    for t, s in samples:
        plant.record(t, s)
    return plant


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
        """InvertedPendulum exposes the PlantProtocol surface."""
        assert callable(plant.derivative)
        assert callable(plant.linear_state_space)
        assert callable(plant.record)
        assert callable(plant.plot)
        assert callable(plant.animate)

    def test_history_starts_empty(self, plant):
        assert plant.t_history == []
        assert plant.state_history == []


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


class TestRecord:
    def test_record_appends_to_history(self, plant):
        state = np.array([[0.1], [0.2], [0.3], [0.4]])
        plant.record(1.5, state)
        assert plant.t_history == [1.5]
        assert len(plant.state_history) == 1
        assert_allclose(plant.state_history[0], [0.1, 0.2, 0.3, 0.4])

    def test_record_flattens_column_vector(self, plant):
        state = np.array([[1.0], [2.0], [3.0], [4.0]])
        plant.record(0.0, state)
        assert plant.state_history[0].shape == (4,)

    def test_record_accepts_1d_state(self, plant):
        state = np.array([1.0, 2.0, 3.0, 4.0])
        plant.record(0.0, state)
        assert_allclose(plant.state_history[0], state)

    def test_record_stores_copy_not_reference(self, plant):
        """Mutating the input array after record must not affect history."""
        state = np.array([1.0, 2.0, 3.0, 4.0])
        plant.record(0.0, state)
        state[0] = 999.0
        assert plant.state_history[0][0] == 1.0

    def test_record_does_not_mutate_self_state(self, plant):
        """record should not change self.state (it's a recorder, not a setter)."""
        plant.state = np.array([5.0, 6.0, 7.0, 8.0])
        plant.record(0.0, np.array([1.0, 2.0, 3.0, 4.0]))
        assert_allclose(np.asarray(plant.state).reshape(-1), [5.0, 6.0, 7.0, 8.0])

    def test_multiple_records_accumulate(self, plant):
        for i in range(5):
            plant.record(float(i), np.array([i, i, i, i], dtype=float))
        assert len(plant.t_history) == 5
        assert len(plant.state_history) == 5
        assert plant.t_history == [0.0, 1.0, 2.0, 3.0, 4.0]


class TestPlot:
    def test_plot_empty_history_returns_empty_list(self, plant):
        assert plant.plot() == []

    def test_plot_returns_four_scatter_traces(self, plant_with_history):
        traces = plant_with_history.plot()
        assert len(traces) == 4
        for tr in traces:
            assert isinstance(tr, go.Scatter)

    def test_plot_trace_names(self, plant_with_history):
        traces = plant_with_history.plot()
        assert [t.name for t in traces] == ["x", "x_dot", "phi", "phi_dot"]

    def test_plot_trace_lengths_match_history(self, plant_with_history):
        traces = plant_with_history.plot()
        n = len(plant_with_history.t_history)
        for tr in traces:
            assert len(tr.x) == n
            assert len(tr.y) == n

    def test_plot_x_uses_t_history(self, plant_with_history):
        traces = plant_with_history.plot()
        assert_allclose(np.array(traces[0].x), np.array(plant_with_history.t_history))

    def test_plot_y_uses_state_column(self, plant_with_history):
        traces = plant_with_history.plot()
        states = np.array(plant_with_history.state_history)
        assert_allclose(np.array(traces[0].y), states[:, 0])
        assert_allclose(np.array(traces[2].y), states[:, 2])


class TestAnimate:
    def test_animate_empty_history_returns_empty_figure(self, plant):
        fig = plant.animate()
        assert isinstance(fig, go.Figure)
        assert len(fig.frames) == 0

    def test_animate_returns_figure_with_frames(self, plant_with_history):
        fig = plant_with_history.animate()
        assert isinstance(fig, go.Figure)
        assert len(fig.frames) > 0

    def test_animate_frame_count_respects_max_frames(self, plant):
        # 250 samples, max_frames=30 -> stride = ceil(250/30) = 9 -> 28 frames
        for i in range(250):
            plant.record(float(i) * 0.01, np.array([i * 0.01, 0.0, 0.1, 0.0]))
        fig = plant.animate(max_frames=30)
        assert len(fig.frames) <= 30
        assert len(fig.frames) > 0

    def test_animate_default_max_frames_caps_at_120(self, plant):
        # 250 samples with default max_frames=120 -> stride = ceil(250/120) = 3
        for i in range(250):
            plant.record(float(i) * 0.01, np.array([i * 0.01, 0.0, 0.1, 0.0]))
        fig = plant.animate()
        assert len(fig.frames) <= 120
        assert len(fig.frames) > 0

    def test_animate_fewer_samples_than_max_frames_uses_all(self, plant):
        # 4 samples, default max_frames=120 -> stride = 1 -> all 4 frames
        for i in range(4):
            plant.record(float(i) * 0.01, np.array([i * 0.01, 0.0, 0.1, 0.0]))
        fig = plant.animate()
        assert len(fig.frames) == 4

    def test_animate_frame_has_cart_and_pendulum_traces(self, plant_with_history):
        fig = plant_with_history.animate()
        frame = fig.frames[0]
        # Each frame should have a cart trace and a pendulum trace
        assert len(frame.data) == 2
        cart, pendulum = frame.data
        # Cart is a single marker at (x, 0)
        assert len(cart.x) == 1
        assert cart.x[0] == pytest.approx(0.0)
        assert len(cart.y) == 1
        assert cart.y[0] == pytest.approx(0.0)
        # Pendulum line goes from (x, 0) to (bob_x, bob_y)
        assert len(pendulum.x) == 2
        assert len(pendulum.y) == 2

    def test_animate_upright_phi_zero_bob_above_cart(self, plant):
        """At phi=0 (upright) the bob should be at (x, l) — directly above cart."""
        l = plant.length_arm
        plant.record(0.0, np.array([0.0, 0.0, 0.0, 0.0]))
        fig = plant.animate()
        frame = fig.frames[0]
        _, pendulum = frame.data
        bob_x, bob_y = pendulum.x[1], pendulum.y[1]
        assert bob_x == pytest.approx(0.0, abs=1e-9)
        assert bob_y == pytest.approx(l, abs=1e-9)

    def test_animate_positive_phi_bob_left_of_cart(self, plant):
        """phi > 0 is a LEFT tilt (matches ``derivative``): the bob must be
        drawn at x < cart_x so the cart visibly chases the lean as it balances.
        Regression test for the sign convention in ``animate``.
        """
        l = plant.length_arm
        plant.record(0.0, np.array([0.0, 0.0, 0.5, 0.0]))
        fig = plant.animate()
        frame = fig.frames[0]
        cart, pendulum = frame.data
        cart_x = cart.x[0]
        bob_x = pendulum.x[1]
        assert bob_x < cart_x
        # bob_x = x - l*sin(phi); for phi=0.5, x=0 -> bob_x = -l*sin(0.5)
        assert bob_x == pytest.approx(-l * np.sin(0.5))

    def test_animate_negative_phi_bob_right_of_cart(self, plant):
        """phi < 0 is a RIGHT tilt: the bob must be drawn at x > cart_x."""
        l = plant.length_arm
        plant.record(0.0, np.array([0.0, 0.0, -0.5, 0.0]))
        fig = plant.animate()
        frame = fig.frames[0]
        cart, pendulum = frame.data
        cart_x = cart.x[0]
        bob_x = pendulum.x[1]
        assert bob_x > cart_x
        assert bob_x == pytest.approx(-l * np.sin(-0.5))

    def test_animate_bob_y_uses_cos_phi(self, plant):
        """bob_y = l*cos(phi) regardless of the x-offset sign; phi=pi/2 -> y=0."""
        l = plant.length_arm
        plant.record(0.0, np.array([0.0, 0.0, np.pi / 2, 0.0]))
        fig = plant.animate()
        frame = fig.frames[0]
        _, pendulum = frame.data
        bob_y = pendulum.y[1]
        assert bob_y == pytest.approx(0.0, abs=1e-9)

    def test_animate_cart_and_bob_both_on_lean_side(self, plant):
        """Sign-consistency regression: with phi > 0 (left tilt) and a cart
        position on the left (as a balancing controller would produce), the
        bob must also be on the left and further left than the cart — so the
        animation reads as the cart chasing the lean.

        Before the sign fix, the bob was drawn on the opposite side from the
        dynamics, so the cart appeared to flee the lean.
        """
        plant.record(0.0, np.array([-0.1, 0.0, 0.5, 0.0]))
        fig = plant.animate()
        frame = fig.frames[0]
        cart, pendulum = frame.data
        cart_x = cart.x[0]
        bob_x = pendulum.x[1]
        assert cart_x == pytest.approx(-0.1)
        # Both cart and bob are on the left side of the origin (matching lean)
        assert bob_x < cart_x  # bob further left than cart
        assert bob_x < 0.0

    def test_animate_layout_has_play_and_reset_buttons(self, plant_with_history):
        fig = plant_with_history.animate()
        assert fig.layout.updatemenus is not None
        assert len(fig.layout.updatemenus) >= 1
        buttons = fig.layout.updatemenus[0].buttons
        labels = [b.label for b in buttons]
        assert "Play" in labels
        assert "Reset" in labels

    def test_animate_reset_button_targets_first_frame(self, plant_with_history):
        fig = plant_with_history.animate()
        buttons = fig.layout.updatemenus[0].buttons
        reset = next(b for b in buttons if b.label == "Reset")
        # args[0] is the frame selector; should target the first frame's name.
        # plotly converts lists to tuples internally, so compare as tuples.
        assert tuple(reset.args[0]) == (fig.frames[0].name,)
        # args[1] should request an immediate jump (no transition duration)
        assert reset.args[1]["mode"] == "immediate"
        assert reset.args[1]["frame"]["duration"] == 0

    def test_animate_play_button_plays_from_current(self, plant_with_history):
        fig = plant_with_history.animate()
        buttons = fig.layout.updatemenus[0].buttons
        play = next(b for b in buttons if b.label == "Play")
        # args[0] is None → play all frames; fromcurrent=True → resume from slider
        assert play.args[0] is None
        assert play.args[1]["fromcurrent"] is True

    def test_animate_layout_has_slider(self, plant_with_history):
        fig = plant_with_history.animate()
        assert fig.layout.sliders is not None
        assert len(fig.layout.sliders) >= 1

    def test_animate_slider_step_count_matches_frames(self, plant_with_history):
        fig = plant_with_history.animate()
        slider = fig.layout.sliders[0]
        assert len(slider.steps) == len(fig.frames)

    def test_animate_slider_step_targets_frame_by_name(self, plant_with_history):
        fig = plant_with_history.animate()
        slider = fig.layout.sliders[0]
        # Each step's args[0] should contain the matching frame name. Plotly
        # converts lists to tuples internally, so compare as tuples.
        for i, step in enumerate(slider.steps):
            assert tuple(step.args[0]) == (fig.frames[i].name,)
            assert step.args[1]["mode"] == "immediate"

    def test_animate_slider_starts_at_first_frame(self, plant_with_history):
        fig = plant_with_history.animate()
        slider = fig.layout.sliders[0]
        assert slider.active == 0

    def test_animate_slider_currentvalue_prefix_is_time(self, plant_with_history):
        fig = plant_with_history.animate()
        slider = fig.layout.sliders[0]
        assert slider.currentvalue["prefix"] == "Time: "

    def test_animate_play_button_duration_matches_sim_time_per_frame(
        self, plant_with_history
    ):
        """Play button frame.duration must equal the real sim-time-per-frame
        in ms so playback wall-clock ≈ t_final (not a hardcoded value)."""
        fig = plant_with_history.animate()
        # Recompute the subsampled t the same way animate() does
        t_full = np.array(plant_with_history.t_history)
        stride = max(1, math.ceil(len(t_full) / 120))
        t = t_full[::stride]
        expected_ms = float(np.mean(np.diff(t))) * 1000 if len(t) > 1 else 0.0

        buttons = fig.layout.updatemenus[0].buttons
        play = next(b for b in buttons if b.label == "Play")
        assert play.args[1]["frame"]["duration"] == pytest.approx(expected_ms)

    def test_animate_playback_walltime_approximates_t_final(self, plant):
        """Total wall playback (frames * duration_ms) ≈ t_final across a range
        of sim lengths. Regression for the hardcoded-50ms bug where a 2s sim
        and a 20s sim both played in ~5-6s."""
        for t_final in (2.0, 10.0, 20.0):
            plant.t_history = []
            plant.state_history = []
            dt = 0.01
            n = int(t_final / dt)
            for i in range(n):
                plant.record(float(i) * dt, np.array([i * dt, 0.0, 0.1, 0.0]))
            fig = plant.animate()
            n_frames = len(fig.frames)
            duration_ms = (
                fig.layout.updatemenus[0].buttons[0].args[1]["frame"]["duration"]
            )
            wall = n_frames * duration_ms / 1000.0
            # t_final is the last sample time; allow 5% tolerance for stride rounding
            assert wall == pytest.approx(t_final, rel=0.05), (
                f"t_final={t_final}: wall={wall:.3f}s, n_frames={n_frames}, "
                f"duration_ms={duration_ms}"
            )

    def test_animate_single_frame_duration_zero_no_crash(self, plant):
        """One recorded sample -> no diff available -> duration 0, no exception."""
        plant.record(0.0, np.array([0.0, 0.0, 0.0, 0.0]))
        fig = plant.animate()
        buttons = fig.layout.updatemenus[0].buttons
        play = next(b for b in buttons if b.label == "Play")
        assert play.args[1]["frame"]["duration"] == 0
        assert len(fig.frames) == 1

    def test_animate_slider_every_step_labeled_with_time(self, plant):
        """Every slider step is labeled with its time so the slider's
        currentvalue readout is never blank as you scrub."""
        for i in range(40):
            plant.record(float(i) * 0.05, np.array([i * 0.01, 0.0, 0.1, 0.0]))
        fig = plant.animate()
        slider = fig.layout.sliders[0]
        # Recompute subsampled t to know expected labels
        t_full = np.array(plant.t_history)
        stride = max(1, math.ceil(len(t_full) / 120))
        t = t_full[::stride]
        for i, step in enumerate(slider.steps):
            assert step.label == f"{t[i]:.2f}"
            assert step.label != ""

    def test_animate_slider_all_labeled_for_few_frames(self, plant):
        # 4 frames -> every step labeled with its time
        for i in range(4):
            plant.record(float(i) * 0.01, np.array([i * 0.01, 0.0, 0.1, 0.0]))
        fig = plant.animate()
        slider = fig.layout.sliders[0]
        labels = [s.label for s in slider.steps]
        assert all(lb != "" for lb in labels)
        assert len(labels) == 4

    def test_animate_layout_has_axis_ranges(self, plant_with_history):
        fig = plant_with_history.animate()
        # ranges should be set (not None) so the view doesn't jump frame-to-frame
        assert fig.layout.xaxis.range is not None
        assert fig.layout.yaxis.range is not None

    def test_animate_y_range_based_on_bar_length(self, plant_with_history):
        """y extents are a function of the bar length l only (with 20% padding)."""
        l = plant_with_history.length_arm
        fig = plant_with_history.animate()
        y_min, y_max = fig.layout.yaxis.range
        assert y_min == pytest.approx(-1.2 * l)
        assert y_max == pytest.approx(1.2 * l)

    def test_animate_y_range_independent_of_state_data(self):
        """y extents must not depend on the recorded state history — only on l.

        Two plants with the same l but very different state trajectories
        (one mild, one with large cart travel) must produce identical y ranges.
        """
        p1 = InvertedPendulum(
            mass_cart=0.5,
            mass_arm=0.2,
            length_arm=0.3,
            friction=0.1,
            gravity=9.8,
            state=np.zeros(4),
        )
        p2 = InvertedPendulum(
            mass_cart=0.5,
            mass_arm=0.2,
            length_arm=0.3,
            friction=0.1,
            gravity=9.8,
            state=np.zeros(4),
        )
        # Mild trajectory
        for i in range(20):
            p1.record(i * 0.01, np.array([0.01 * i, 0.0, 0.1, 0.0]))
        # Wild trajectory with large cart travel and big angles
        for i in range(20):
            p2.record(i * 0.01, np.array([5.0 * i, 0.0, 1.5, 0.0]))

        fig1 = p1.animate()
        fig2 = p2.animate()
        assert fig1.layout.yaxis.range == fig2.layout.yaxis.range

    def test_animate_y_range_scales_with_bar_length(self):
        """A longer bar must produce a proportionally larger y range."""

        def make_fig(length):
            p = InvertedPendulum(
                mass_cart=0.5,
                mass_arm=0.2,
                length_arm=length,
                friction=0.1,
                gravity=9.8,
                state=np.zeros(4),
            )
            p.record(0.0, np.array([0.0, 0.0, 0.1, 0.0]))
            return p.animate()

        short = make_fig(0.1)
        long = make_fig(1.0)
        short_span = short.layout.yaxis.range[1] - short.layout.yaxis.range[0]
        long_span = long.layout.yaxis.range[1] - long.layout.yaxis.range[0]
        # span = 2.4 * l, so the ratio equals the length ratio
        assert long_span / short_span == pytest.approx(1.0 / 0.1)

    def test_animate_x_axis_not_scaleanchored_to_y(self, plant_with_history):
        """x axis must not be scale-anchored to y, otherwise plotly expands the
        y view to maintain aspect ratio when the cart travels far — breaking
        the 'constant y extents' guarantee."""
        fig = plant_with_history.animate()
        # No scaleanchor set on xaxis
        assert fig.layout.xaxis.scaleanchor is None


class TestDynamicsValidationAgainstCTMS:
    """Independent validation of plant dynamics against the canonical CTMS
    inverted-pendulum reference
    (ctms.engin.umich.edu/CTMS, InvertedPendulum, SystemModeling).

    Unlike TestLinearStateSpace::test_A_B_known_values_for_default_params,
    these tests pin the dynamics to published / physically-derived values
    rather than re-deriving the same closed-form expressions the code uses.
    """

    def test_linear_state_space_matches_CTMS_published_values(self, plant):
        """A and B match the values published in the CTMS state-space example
        for M=0.5, m=0.2, l=0.3, b=0.1, g=9.8, I=0.006.

        Tolerances are set to the rounding precision of the published values
        (CTMS reports 4 significant figures), so a real bug would be caught
        while rounding noise in the reference is tolerated.
        """
        A, B, _, _ = plant.linear_state_space()
        # CTMS published A row 2: [0, -0.1818, 2.673, 0]
        assert_allclose(A[1, 1], -0.1818, atol=5e-5)
        assert_allclose(A[1, 2], 2.673, atol=5e-4)
        # CTMS published A row 4: [0, -0.4545, 31.18, 0]
        assert_allclose(A[3, 1], -0.4545, atol=5e-5)
        assert_allclose(A[3, 2], 31.18, atol=5e-3)
        # CTMS published B: [0; 1.818; 0; 4.545]
        assert_allclose(B[1, 0], 1.818, atol=5e-4)
        assert_allclose(B[3, 0], 4.545, atol=5e-4)

    def test_open_loop_has_unstable_eigenvalue_near_sqrt_A43(self, plant):
        """The unstable open-loop pole for the inverted pendulum is
        approximately sqrt(mgl(M+m)/P) = sqrt(A43) ~= 5.58 rad/s when friction
        is small. With b=0.1, the friction-induced shift is ~0.4%."""
        A, _, _, _ = plant.linear_state_space()
        eigs = np.linalg.eigvals(A)
        # Filter out numerical noise near zero; only one true RHP eigenvalue.
        positive_real = eigs.real[eigs.real > 1e-6]
        assert len(positive_real) == 1
        unstable = positive_real[0]
        expected = np.sqrt(A[3, 2])  # sqrt(A43)
        assert_allclose(unstable, expected, rtol=0.01)
        # Sanity: clearly unstable (well above zero)
        assert unstable > 5.0

    def test_energy_conserved_without_friction_or_force(self):
        """A frictionless, force-free pendulum must conserve total mechanical
        energy to integration tolerance. Validates the nonlinear EOM conserve
        energy (the Lagrangian is consistent with the mass matrix / RHS)."""
        from scipy.integrate import solve_ivp

        state = np.array([0.0, 0.0, 0.2, 0.0])
        plant = InvertedPendulum(
            mass_cart=0.5,
            mass_arm=0.2,
            length_arm=0.3,
            friction=0.0,
            gravity=9.8,
            state=state,
        )
        M, m, l, g, I = (
            plant.mass_cart,
            plant.mass_arm,
            plant.length_arm,
            plant.gravity,
            plant.arm_moment,
        )

        def energy(s):
            x, xd, phi, phid = s
            theta = phi + np.pi
            KE = (
                0.5 * (M + m) * xd**2
                + m * l * np.cos(theta) * xd * phid
                + 0.5 * (I + m * l**2) * phid**2
            )
            PE = -m * g * l * np.cos(theta)
            return KE + PE

        sol = solve_ivp(
            lambda t, y: plant.derivative(y, 0.0),
            (0, 2.0),
            state,
            method="LSODA",
            max_step=0.001,
        )
        E0 = energy(state)
        max_drift = max(abs(energy(sol.y[:, i]) - E0) for i in range(sol.t.size))
        assert max_drift < 1e-5

    def test_nonlinear_derivative_matches_linearization_near_upright(self, plant):
        """For small ||s|| the nonlinear derivative must match A@s to high
        precision -- the linearization is the Jacobian of the nonlinear
        dynamics about the upright equilibrium. Error scales as O(||s||^3),
        so eps=1e-3 gives ~1e-8 error and atol=1e-7 has ~1 decade of margin."""
        A, _, _, _ = plant.linear_state_space()
        rng = np.random.default_rng(42)
        for _ in range(5):
            s = rng.uniform(-1e-3, 1e-3, size=4)
            nonlinear = plant.derivative(s, 0.0)
            linear = A @ s
            assert_allclose(nonlinear, linear, atol=1e-7)

    def test_hanging_down_equilibrium_returns_zero_derivative(self, plant):
        """At phi = -pi (pendulum hanging straight down) with zero velocity and
        zero force, the system is at a stable equilibrium, so the derivative
        is zero. Mirrors test_at_upright_equilibrium_returns_zeros for the
        second equilibrium of the system and pins the pi-offset convention."""
        hanging = np.array([0.0, 0.0, -np.pi, 0.0])
        deriv = plant.derivative(hanging, 0.0)
        assert_allclose(deriv, np.zeros(4), atol=1e-12)

    def test_gravity_sign_falls_away_from_upright(self, plant):
        """Tilted right of upright (phi > 0) with zero velocity and zero force
        must accelerate further right (phi_ddot > 0); tilted left must
        accelerate left. Confirms upright is unstable and gravity acts in the
        expected direction (not inverted)."""
        deriv_right = plant.derivative(np.array([0.0, 0.0, 0.1, 0.0]), 0.0)
        assert deriv_right[3] > 0
        deriv_left = plant.derivative(np.array([0.0, 0.0, -0.1, 0.0]), 0.0)
        assert deriv_left[3] < 0
