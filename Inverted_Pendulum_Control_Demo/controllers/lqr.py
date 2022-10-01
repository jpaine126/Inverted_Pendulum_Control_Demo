#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 25 17:28:04 2020

@author: jpaine

LQR Controller Class

This class if for implementing a LQR controller, not for designing one.



"""

from math import copysign


class LQR:
    def __init__(self, K, max_input=1, max_input_on=False):
        self.K = K
        self.max_input = max_input
        self.max_input_on = max_input_on

    def set_gain(self, K):
        self.K = K

    def max_input_enable(self):
        self.max_input_on = True

    def max_input_disable(self):
        self.max_input_on = False

    def update(self, states):
        """Update method to calculate force from state."""
        control = 0

        for i in range(0, len(states)):
            control += states[i, 0] * self.K[0, i]
            # print(states[i,0])
            # print(self.K[0,1])

        if self.max_input_on:
            if abs(control) > self.max_input:
                control = copysign(self.max_input, control)

        return -control
