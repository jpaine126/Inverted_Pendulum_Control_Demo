from typing import Literal, Protocol

import numpy as np
import param
from plotly import graph_objects as go

from ..plant import PlantProtocol


class TestSetup(Protocol):
    _is_abstract = True
    _dynamic_type: Literal["observer", "controller"]
    _implemented_observers = dict()
    _implemented_controllers = dict()

    params: dict[str, param.Parameter]

    def __init_subclass__(cls, name=None, is_abstract=False, **kwargs):
        """Register implementations of this class when subclassed."""
        cls._is_abstract = is_abstract

        if name is None and not is_abstract:
            raise TypeError(
                "TestSetup.__init_subclass__() missing 1 required positional argument: 'name'"
            )

        if not cls._is_abstract:
            cls_type = cls._dynamic_type
            if cls_type == "observer":
                dict_to_update = cls._implemented_observers
            else:
                dict_to_update = cls._implemented_controllers
            dict_to_update[name] = cls

    def __init__(self, plant: PlantProtocol, sim_params, **kwargs):
        """Protocol for classes used as controllers and oberservers.

        Used for containing logic for parameters and architecture for specific
        controller and observer designs, including plots.

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
            **kwargs: Arguments required for creation of the underlying controllers
                and observers. Values specified in ``params`` will be passed in.

        """

    def update(self): ...

    def plot(self) -> go.Figure: ...


class ObserverTestSetup(TestSetup, is_abstract=True):
    _dynamic_type = "observer"

    def update(self, control_force: float, state: np.ndarray) -> np.ndarray:
        """Calcualte filtered states from measured states and inputs."""


class ControllerTestSetup(TestSetup, is_abstract=True):
    _dynamic_type = "controller"

    def update(self, state: np.ndarray) -> float:
        """Calcualte output force from states."""


# import all test setups here for registration
from .lqr1 import LQR1
from .pass_through_observer import PassThroughObserver
from .basic_kalman_filter import BasicKalmanFilter
