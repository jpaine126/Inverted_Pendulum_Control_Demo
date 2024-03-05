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

    F - State Transition Matrix
    B - Control Input Matrix
    H - Observation Matrix
    Q - Covariance of the Process Noise
    R - Covariance of the Measurement Noise

    x_pri - a priori state estimate
    P_pri - a priori estimate covariance

    z     - measurement
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
        self.x_last = np.zeros([np.size(F, 1), 1])
        self.P_last = np.zeros(np.shape(F))

        # print("x:", self.x_last)

    def __repr__(self):
        return f"KF({self.F=} {self.B=} {self.H=} {self.Q} {self.R})"

    def update(self, control_force, state):
        """Update method to calculate states."""
        # print("H:", self.H, "size:", self.H.shape)
        # print("state:", state, "size:", state.shape)

        measurement = np.matmul(self.H, state)   
        # print("measurement:", measurement) 
        control_force = np.array(control_force)
        measurement = np.array(measurement)

        ## Predict Step
        # x_k|k-1 = F*x_k-1|k-1 + B*u
        x_pri = np.matmul(self.F, self.x_last) + self.B * control_force

        # P_k|k-1 = F*P_k-1|k-1*F' + Q
        P_pri = np.matmul(np.matmul(self.F, self.P_last), np.transpose(self.F)) + self.Q

        ## Update Step
        # y_k = z_k - H*x_k|k-1
        y_hat = measurement - np.matmul(self.H, x_pri)

        # S = H*P_k|k-1*H' + R
        S = np.matmul(np.matmul(self.H, P_pri), np.transpose(self.H)) + self.R

        # K = P_k|k-1*H'*inv(S)
        K = np.matmul(np.matmul(P_pri, np.transpose(self.H)), inv(S))

        # x_k|k = x_k|k-1 + K*y_k
        x_post = x_pri + np.matmul(K, y_hat)

        # P_k|k = (I - K*H)*P_k|k-1
        P_post = np.matmul((np.eye(4) - np.matmul(K, self.H)), P_pri)

        # y_k|k = z - H*x_k|k
        # y = z - np.matmul(self.H, x_post)

        ## Store for next step

        self.x_last = x_post
        self.P_last = P_post

        return x_post
