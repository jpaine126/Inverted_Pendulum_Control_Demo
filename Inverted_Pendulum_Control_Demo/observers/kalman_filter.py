#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 25 17:28:04 2020

@author: jpaine
"""
import numpy as np
from numpy.linalg import inv


class KalmanFilter:
    """Kalman filter implementation.

    Very basic implementation assuming only position input.

    F - State Transition Matrix (A matrix from our example)
    B - Control Input Matrix
    H - Observation Matrix (C matrix from our example)
    Q - Covariance of the Process Noise
    R - Covariance of the Measurement Noise

    x_pri - a priori state estimate
    P_pri - a priori estimate covariance

    z     - measurement (y vector from our example)
    u     - control force

    y_hat - measurement pre-fit residual
    S     - prefir residual covarience
    K     - Kalman gain
    x_post- updated state estimate
    P_post- updated estiamte covariance
    y     - measurement post-fit residual
    """

    def __init__(self, F, B, H, Q, R):
        self.F = np.array(F)
        self.B = np.array(B)
        self.H = np.array(H)
        self.Q = np.array(Q)
        self.R = np.array(R)
        self.x_last = np.zeros([np.size(F, 1), 1]) # x_k-1|k-1
        self.P_last = np.zeros(np.shape(F)) # P_k-1|k-1

        # print("x:", self.x_last)

    def __repr__(self):
        return f"KF({self.F=} {self.B=} {self.H=} {self.Q} {self.R})"

    def update(self, control_force, state):
        """Update method to calculate states."""
        control_force = np.array(control_force)

        ## Predict Step
        # x_k|k-1 = F*x_k-1|k-1 + B*u
        x_pri = ... # TODO (use np.matmul() for matrix multiplication)

        # P_k|k-1 = F*P_k-1|k-1*F' + Q
        P_pri = ... # TODO (use np.matmul() for matrix multiplication)

        ## Update Step
        ## Update Step
        measurement = np.matmul(self.H, state)  
        measurement = np.array(measurement) # z_k

        # y_k = z_k - H*x_k|k-1
        y_hat = ... # TODO (use np.matmul() for matrix multiplication)

        # S = H*P_k|k-1*H' + R
        S = np.matmul(np.matmul(self.H, P_pri), np.transpose(self.H)) + self.R

        # K = P_k|k-1*H'*inv(S)
        K = ... # TODO (use np.matmul() for matrix multiplication)

        # x_k|k = x_k|k-1 + K*y_k
        x_post = x_pri + np.matmul(K, y_hat)

        # P_k|k = (I - K*H)*P_k|k-1
        P_post = np.matmul((np.eye(4) - np.matmul(K, self.H)), P_pri)


        ## Store for next step

        self.x_last = x_post
        self.P_last = P_post

        return x_post
