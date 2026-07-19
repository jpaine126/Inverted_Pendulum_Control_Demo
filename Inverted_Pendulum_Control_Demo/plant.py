import math
from typing import List, Protocol

import numpy as np
from plotly import graph_objects as go


class PlantProtocol(Protocol):
    """Plant class to handle physics and integration.

    All state matrices will be kept in discrete form. If dt is provided, the
    state matrices must be given in continuous form. dt is not used for the
    actual propagation because a constant dt is assumed.
    """

    def derivative(self, state: np.ndarray, force: np.ndarray) -> np.ndarray:
        """Derivatives of full EQs of motion.

        Used for numerical integration for non-linear simulation.
        """

    def linear_state_space(self):
        """Linear state space equations in continuous form.

        Not used in simulatino loop. Available for controllers and observers that want
        to design to the linear model.
        """

    def record(self, time: float, state: np.ndarray) -> None:
        """Append a (time, state) sample to the plant's history.

        Called by the sim loop each control step so the plant can build its own
        record for plotting and animation after the sim has run.
        """

    def plot(self) -> List[go.Scatter]:
        """Time-series traces of the plant's states.

        Returns a list of plotly Scatter traces (x, x_dot, phi, phi_dot) vs time
        for inclusion in the dashboard's main figure.
        """

    def animate(self) -> go.Figure:
        """Animated figure of the plant's motion.

        Returns a plotly Figure with frames showing the cart and pendulum arm
        over the recorded history.
        """


class InvertedPendulum(PlantProtocol):
    def __init__(
        self,
        mass_cart,
        mass_arm,
        length_arm,
        friction,
        gravity,
        state,
    ):
        self.state = state
        self.mass_cart = mass_cart
        self.mass_arm = mass_arm
        self.length_arm = length_arm
        self.cart_friction = friction
        self.gravity = gravity
        self.arm_moment = (1 / 3) * mass_arm * (length_arm**2)

        self.t_history: list = []
        self.state_history: list = []

    def derivative(self, state, force):
        """Derivatives of the full nonlinear EOMs.

        Sign convention: ``state[2]`` (phi) is the angle from upright, with
        phi > 0 corresponding to a tilt to the LEFT. Internally we add pi so
        ``theta = phi + pi`` places the hanging-down rest position at
        ``theta = pi`` (sin = 0); gravity then drives ``phi_ddot > 0`` for
        phi > 0, growing the leftward tilt. Controllers/observers and the
        animation (see ``animate``) all use this same convention.
        """
        M = self.mass_cart
        m = self.mass_arm
        l = self.length_arm
        b = self.cart_friction
        g = self.gravity
        I = self.arm_moment

        x_dot = state[1]
        # phi > 0 = left tilt; theta = phi + pi => hanging-down rest is theta=pi
        theta = state[2] + np.pi
        theta_dot = state[3]

        A = np.array(
            [
                [M + m, m * l * np.cos(theta)],
                [m * l * np.cos(theta), I + m * l**2],
            ]
        )
        B = np.array(
            [
                force - b * x_dot + m * l * (theta_dot**2) * np.sin(theta),
                -m * g * l * np.sin(theta),
            ]
        )

        accels = np.linalg.inv(A) @ B
        return np.hstack((x_dot, accels[0], theta_dot, accels[1]))

    def linear_state_space(self):
        mass_cart = self.mass_cart
        mass_arm = self.mass_arm
        length_arm = self.length_arm
        cart_friction = self.cart_friction
        gravity = self.gravity
        arm_moment = (1 / 3) * mass_arm * (length_arm**2)

        P = arm_moment * (mass_cart + mass_arm) + (mass_cart * mass_arm * length_arm**2)
        A22 = (-(arm_moment + mass_arm * (length_arm**2)) * cart_friction) / P
        A23 = ((mass_arm**2) * gravity * (length_arm**2)) / P
        A42 = (-(mass_arm * length_arm * cart_friction)) / P
        A43 = (mass_arm * gravity * length_arm * (mass_cart + mass_arm)) / P

        A = np.array([[0, 1, 0, 0], [0, A22, A23, 0], [0, 0, 0, 1], [0, A42, A43, 0]])

        B2 = (arm_moment + mass_arm * (length_arm**2)) / P
        B4 = (mass_arm * length_arm) / P

        B = np.array([[0], [B2], [0], [B4]])

        C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])

        D = np.array([[0], [0]])

        return A, B, C, D

    def record(self, time, state):
        """Append a (time, state) sample to the plant's history.

        Stores a flattened copy of ``state`` so later mutations of ``self.state``
        don't corrupt the recorded history.
        """
        flat = np.asarray(state).reshape((-1,))
        self.t_history.append(float(time))
        self.state_history.append(flat.copy())

    def plot(self):
        """Return a list of go.Scatter traces for the four states vs time."""
        if not self.state_history:
            return []
        states = np.array(self.state_history)
        t = np.array(self.t_history)
        names = ["x", "x_dot", "phi", "phi_dot"]
        return [
            go.Scatter(x=t, y=states[:, i], name=name) for i, name in enumerate(names)
        ]

    def animate(self, max_frames=120):
        """Return an animated go.Figure of the cart + pendulum arm.

        Geometry: pivot at the cart position (x, 0). The bob sits at
        (x - l*sin(phi), l*cos(phi)) so phi=0 is upright. The sign on the
        x-offset matches the plant dynamics' convention: phi > 0 is a tilt
        to the LEFT (see ``derivative``), so the bob is drawn to the left of
        the cart and the cart visibly chases the lean as it balances.

        Frames subsample the recorded history to at most ``max_frames`` to
        keep the figure size manageable for long simulations. Playback is
        real-time: each frame's duration is set to the actual sim-time-per-frame
        (in ms), so wall-clock playback ≈ t_final regardless of stride.
        """
        if not self.state_history:
            return go.Figure()

        states = np.array(self.state_history)
        t = np.array(self.t_history)
        n = len(t)
        # ceiling division so the frame count is guaranteed <= max_frames
        stride = max(1, math.ceil(n / max_frames))
        t = t[::stride]
        states = states[::stride]

        l = self.length_arm
        x_cart = states[:, 0]
        phi = states[:, 2]
        # phi > 0 = left tilt (see derivative), so bob is left of cart: x - l*sin(phi)
        bob_x = x_cart - l * np.sin(phi)
        bob_y = l * np.cos(phi)

        # Real sim-time-per-frame in ms, so playback wall-clock ≈ t_final.
        # Single-frame guard: no diff available -> duration 0 (static frame).
        frame_duration_ms = float(np.mean(np.diff(t))) * 1000 if len(t) > 1 else 0.0

        frames = []
        for i in range(len(t)):
            xi = float(x_cart[i])
            bx = float(bob_x[i])
            by = float(bob_y[i])
            frames.append(
                go.Frame(
                    data=[
                        go.Scatter(
                            x=[xi],
                            y=[0],
                            mode="markers",
                            marker=dict(size=14, symbol="square", color="blue"),
                            name="cart",
                        ),
                        go.Scatter(
                            x=[xi, bx],
                            y=[0, by],
                            mode="lines+markers",
                            line=dict(color="black", width=2),
                            marker=dict(size=10, color="red"),
                            name="pendulum",
                        ),
                    ],
                    name=f"{t[i]:.3f}",
                )
            )

        fig = go.Figure(data=frames[0].data, frames=frames)

        x_min = float(min(x_cart.min(), bob_x.min())) - 0.2
        x_max = float(max(x_cart.max(), bob_x.max())) + 0.2
        # Y extents are constant and based only on the bar length l: the bob
        # lives in [-l, +l] (y = l*cos(phi)), so pad by 20% of l on each side.
        # No scaleanchor on x so y stays independent of the cart's travel.
        y_pad = 0.2 * l
        y_min = -l - y_pad
        y_max = l + y_pad

        # Slider: one step per frame for scrubbing. Every step is labeled with
        # its time so currentvalue always shows the time as you scrub.
        slider_steps = [
            dict(
                label=f"{t[i]:.2f}",
                method="animate",
                args=[
                    [frames[i].name],
                    dict(mode="immediate", frame=dict(duration=0, redraw=False)),
                ],
            )
            for i in range(len(t))
        ]

        fig.update_layout(
            xaxis=dict(range=[x_min, x_max], title="x (m)"),
            yaxis=dict(range=[y_min, y_max], title="y (m)"),
            title="Inverted Pendulum Animation",
            updatemenus=[
                dict(
                    type="buttons",
                    buttons=[
                        dict(
                            label="Play",
                            method="animate",
                            args=[
                                None,
                                dict(
                                    frame=dict(
                                        duration=frame_duration_ms, redraw=False
                                    ),
                                    fromcurrent=True,
                                ),
                            ],
                        ),
                        dict(
                            label="Reset",
                            method="animate",
                            args=[
                                [frames[0].name],
                                dict(
                                    mode="immediate",
                                    frame=dict(duration=0, redraw=False),
                                ),
                            ],
                        ),
                    ],
                )
            ],
            sliders=[
                dict(
                    active=0,
                    currentvalue={"prefix": "Time: "},
                    pad={"b": 10, "t": 50},
                    steps=slider_steps,
                )
            ],
        )
        return fig
