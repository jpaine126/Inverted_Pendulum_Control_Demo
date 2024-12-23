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
