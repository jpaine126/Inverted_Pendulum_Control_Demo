from .. import Observer


class PassThroughObserver(Observer):
    """Place holder for a pass through observer that copies the whole state."""

    def update(self, control_force, measurement):
        """Update method to calculate states."""
        return measurement
