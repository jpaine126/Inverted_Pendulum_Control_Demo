"""Final implemented designs of controllers and observers."""

from typing import Literal, Protocol

import numpy as np
import param
from plotly import graph_objects as go

from ..plant import PlantProtocol
from ..sim_parameters import ControlDemoParam


class TestSetup(Protocol):
    _is_abstract = True
    _dynamic_type: Literal["observer", "controller"]
    _implemented_observers = dict()
    _implemented_controllers = dict()

    params: dict[str, param.Parameter]

    def __init_subclass__(cls, setup_name=None, is_abstract=False, **kwargs):
        """Register implementations of this class when subclassed.

        Args:
            setup_name: Name of the Test Case implementation. Required for all non-abstract classes.
            is_abstract: Whether or not this subclass is abstract. Used to prevent
                registration for subclasses that aren't meant to be a final implementation.
        """
        cls._is_abstract = is_abstract

        if setup_name is None and not is_abstract:
            raise TypeError(
                "TestSetup.__init_subclass__() missing 1 required positional argument: 'name'"
            )

        if not cls._is_abstract:
            cls_type = cls._dynamic_type
            if cls_type == "observer":
                dict_to_update = cls._implemented_observers
            elif cls_type == "controller":
                dict_to_update = cls._implemented_controllers
            else:
                raise ValueError(f"Invalid _dynamic_type {cls_type} provided for {cls}")
            dict_to_update[setup_name] = cls

    def __init__(self, plant: PlantProtocol, sim_params: ControlDemoParam, **kwargs):
        """Protocol for classes used as controllers and observers.

        Used for containing logic for parameters and architecture for specific
        controller and observer designs, including plots.

        Warning:
            This class provides common logic for observers and controllers. The ObserverTestSetup
            and ControllerTestSetup subclasses should be subclassed from instead of this class,
            as they provide the actual interfaces used by the main sim loop for both types.

        Attributes:
            _is_abstract: Whether or not this class is a final defintion to be used in the sim.
            _dynamic_type: Type of TestSetup. Used for directing registration of subclasses.
            _implemented_observers: All observer test setups implemented, as a dict of name
                (passed as an arg when subclassing) to subclass.
            _implemented_controllers: All controller test setups implemented, as a dict of name
                (passed as an arg when subclassing) to subclass.
            params: Dict of parameters defining the test setup.

        Args:
            plant: The plant object being used.
            sim_params: Simulation-wide parameters.
            **kwargs: Arguments required for creation of the underlying controllers
                and observers. Values specified in ``params`` will be passed in.

        """

    def plot(self) -> dict[str, go.Figure]:
        """Return a dict of plotly Figures for the dashboard.

        Implementations should append (time, value) pairs in ``update()`` and
        build one or more Scatter traces here, returning them wrapped in a
        ``go.Figure`` (with axis titles) keyed by a figure name so the dashboard
        can assign each pane directly without further trace composition. An
        empty history yields an empty dict.
        """

    def update(self): ...


class ObserverTestSetup(TestSetup, is_abstract=True):
    _dynamic_type = "observer"

    # Indices of the plant's [x, x_dot, phi, phi_dot] within the observer's
    # internal state vector. ``None`` means the internal state *is* the plant
    # state (identity, 4D); a list (e.g. [0, 1, 3, 4] for a constant-acceleration
    # filter with internal [x, x_dot, x_ddot, phi, phi_dot, phi_ddot]) lets an
    # observer estimate a richer state internally while still handing the
    # controller exactly the plant's 4 states.
    plant_state_indices: list[int] | None = None
    # Dimension of the observer's internal state. Required when
    # ``plant_state_indices`` is set so the injected measurement can be sized
    # (a trailing acceleration state, e.g. phi_ddot at index 5, would be missed
    # by ``max(plant_state_indices) + 1``). Ignored when ``plant_state_indices``
    # is ``None``.
    internal_state_dim: int | None = None

    def _inject_state(self, state: np.ndarray) -> np.ndarray:
        """Map a 4D plant measurement into the observer's internal state space.

        Plant-state slots are filled from ``state`` (the measured x, x_dot,
        phi, phi_dot); all other slots are zero. The non-plant slots are
        ignored by the observation matrix ``H`` (which only selects x and phi),
        so the fill value is mathematically irrelevant; it just gives the
        internal measurement the right shape. Returns ``state`` unchanged when
        ``plant_state_indices`` is ``None`` (identity).
        """
        if self.plant_state_indices is None:
            return state
        internal = np.zeros((self.internal_state_dim, 1))
        internal[self.plant_state_indices] = np.asarray(state).reshape((-1, 1))
        return internal

    def _extract_state(self, estimate: np.ndarray) -> np.ndarray:
        """Extract the 4D plant state from an ND internal estimate.

        Selects the rows at ``plant_state_indices``. Returns ``estimate``
        unchanged when ``plant_state_indices`` is ``None`` (identity). Always
        returns a 2-D ``(4, 1)`` array so the controller and the sim recorder
        see a consistent shape regardless of the observer's internal DOF.
        """
        if self.plant_state_indices is None:
            return estimate
        return np.asarray(estimate).reshape((-1, 1))[self.plant_state_indices]

    def update(
        self, control_force: float, state: np.ndarray, time: float
    ) -> np.ndarray:
        """Calculate filtered states from measured states and inputs."""


class ControllerTestSetup(TestSetup, is_abstract=True):
    _dynamic_type = "controller"

    def update(self, state: np.ndarray, time: float) -> float:
        """Calculate output force from states."""


# import all test setups here for registration
from .basic_pid import BasicPID
from .ca_kalman_filter import CAKalmanFilter
from .dynamic_kalman_filter import DynamicKalmanFilter
from .lqr1 import LQR1
from .pass_through_observer import PassThroughObserver
