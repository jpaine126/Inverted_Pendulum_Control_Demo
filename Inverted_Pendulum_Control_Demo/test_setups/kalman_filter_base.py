import numpy as np
import pandas as pd
from plotly import graph_objects as go
from scipy import signal

from ..plant import PlantProtocol
from ..primitives.observers import KalmanFilter
from . import ObserverTestSetup


class KalmanFilterBase(ObserverTestSetup, is_abstract=True):
    """Implements Kalman Filter update and plot methods with no filter design."""

    # Names of the observer's internal state components, used to label the
    # per-state traces in the ``"Estimates"`` figure. ``None`` means the
    # internal state is the 4D plant state ``[x, x_dot, phi, phi_dot]`` and
    # those names are used; subclasses with an augmented internal state (e.g.
    # a constant-acceleration filter) override this with the full name list.
    state_names: list[str] | None = None

    params = dict(
        Q=pd.DataFrame(
            [
                [0.01, 0, 0, 0],
                [0, 0.01, 0, 0],
                [0, 0, 0.01, 0],
                [0, 0, 0, 0.01],
            ],
            index=pd.RangeIndex(0, 4, name="Q"),
        ),
        R=pd.DataFrame(
            [
                [0.1, 0],
                [0, 0.05],
            ],
            index=pd.RangeIndex(0, 2, name="R"),
        ),
    )

    def update(
        self, control_force: float, state: np.ndarray, time: float
    ) -> np.ndarray:
        internal = self._inject_state(state)
        full = self.observer.update(control_force, internal)
        self.estimate_history.append(np.asarray(full).reshape((-1,)).copy())
        self.cov_history.append(np.asarray(self.observer.P_last).copy())
        self.t_history.append(float(time))
        return self._extract_state(full)

    def plot(self):
        """Return figures of the estimated states and estimation errors vs time.

        Returns a dict with an ``"Estimates"`` figure (one trace per estimated
        internal state) and, when the plant's true-state history is available,
        an ``"Errors"`` figure (one trace per estimated state, true −
        estimated) and a ``"NEES"`` figure (normalized estimation error
        squared). An empty history yields an empty dict.

        The NEES figure covers the full internal state space of the filter when
        ``plant_state_indices`` is set: the estimates and covariance are stored
        in the observer's internal (possibly augmented) coordinates, and the
        plant's ``augmented_state_history`` (which adds the true accelerations
        recomputed from the nonlinear equations of motion) is used as the
        ground truth. When ``plant_state_indices`` is ``None`` the filter is
        4-D and the plant's plain ``state_history`` is used.
        """
        if not self.estimate_history:
            return {}
        estimates = np.array(self.estimate_history)
        t = np.array(self.t_history)
        names = self.state_names
        if names is None:
            names = ["x", "x_dot", "phi", "phi_dot"]
        est_fig = go.Figure(
            data=[
                go.Scatter(x=t, y=estimates[:, i], name=f"{name} (est)")
                for i, name in enumerate(names)
            ]
        )
        est_fig.update_layout(xaxis_title="Time (s)", yaxis_title="State")
        figures = {"Estimates": est_fig}

        plant = getattr(self, "plant", None)
        plant_history = getattr(plant, "state_history", None)
        plant_t_history = getattr(plant, "t_history", None)
        if plant_history and plant_t_history:
            if self.plant_state_indices is not None:
                plant_states = np.asarray(plant.augmented_state_history)
            else:
                plant_states = np.array(plant_history)
            plant_t = np.array(plant_t_history)
            cov = np.array(self.cov_history)
            err = plant_states - estimates
            P_inv_err = np.linalg.solve(
                cov, err[..., None]
            )  # (T, n, 1); solves cov @ x = err
            nees = np.einsum("ti,ti->t", err, P_inv_err[..., 0])
            n = min(len(plant_states), len(estimates), len(plant_t))
            if n > 0:
                err_fig = go.Figure(
                    data=[
                        go.Scatter(
                            x=plant_t[:n],
                            y=err[:n, i],
                            name=f"{name} Error",
                        )
                        for i, name in enumerate(names)
                    ]
                )
                err_fig.update_layout(xaxis_title="Time (s)", yaxis_title="Error")
                figures["Errors"] = err_fig

                nees_fig = go.Figure(
                    data=[
                        go.Scatter(
                            x=plant_t[:n],
                            y=nees,
                            name="Normalized Estimation Error^2 (NEES)",
                        )
                    ]
                )
                dof = err.shape[1]
                nees_fig.add_hline(
                    y=dof,
                    line_dash="dash",
                    line_color="black",
                    annotation_text=f"E[NEES] = {dof}",
                    annotation_position="top left",
                )
                nees_fig.update_layout(xaxis_title="Time (s)", yaxis_title="NEES")
                figures["NEES"] = nees_fig

        return figures
