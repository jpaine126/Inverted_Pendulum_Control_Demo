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

    def update(self): ...

    def plot(self) -> go.Figure: ...


class ObserverTestSetup(TestSetup, is_abstract=True):
    _dynamic_type = "observer"

    def update(self, control_force: float, state: np.ndarray) -> np.ndarray:
        """Calculate filtered states from measured states and inputs."""


class ControllerTestSetup(TestSetup, is_abstract=True):
    _dynamic_type = "controller"

    def update(self, state: np.ndarray) -> float:
        """Calculate output force from states."""


from .basic_kalman_filter import BasicKalmanFilter

# import all test setups here for registration
from .basic_pid import BasicPID
from .lqr1 import LQR1
from .pass_through_observer import PassThroughObserver
