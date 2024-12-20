# -*- coding: utf-8 -*-
"""
Created on Sat Oct  1 16:23:06 2022

@author: jpaine
"""


class PassThroughObserver:
    """Place holder for a pass through observer that copies the whole state."""

    def update(self, control_force, measurement):
        """Update method to calculate states."""
        return measurement
