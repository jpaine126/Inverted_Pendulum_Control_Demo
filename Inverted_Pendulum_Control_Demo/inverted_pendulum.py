#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 24 13:36:32 2020

@author: jpaine

Simulation of a Cart and Inverted Pendulum System

Includes options for running with either PID or LQR as the controller, 
adding noise to the sensor reading, and for using a Kalman Filter to 
estimate velocities.
"""

import math
import random

import control
import matplotlib.pyplot as plt
# Libraries
import numpy as np
from controllers import LQR, PID
from observers import KalmanFilter
from scipy import signal

"""   SIMULATION PARAMETERS   """
# Switches

# Selected Control Scheme
control_scheme = 2
#  1 - PID
#  2 - LQR

# Noise
measure_noise = 1
#  0 - No Noise
#  1 - Noise

noise_val = 0.002

# Observers
observer = 1
#  0 - None
#  1 - Kalman Filter

# Physical Parameters

M_Cart = 0.5
M_Arm = 0.2
length = 0.3
b = 0.1
g = 9.8
I = (1 / 3) * M_Arm * (length**2)

# Controller Parameters

#  PID
K_P = 100
K_I = 1
K_D = 5

#  LQR

q11 = 5000
q33 = 100

r = 10

Q = np.diagflat([q11, 0, q33, 0])

R = np.array([r])

# Observer Parameters

# Q - Process Noise

Q_kalman = np.array(
    [[0.01, 0, 0, 0], [0, 0.01, 0, 0], [0, 0, 0.01, 0], [0, 0, 0, 0.01]]
)

# R - Measurement Noise

R_kalman = np.array(
    [[(0.001**4) / 4, (0.001**3) / 2], [(0.001**3) / 2, 0.001**2]]
)

# Initialize States
x = 0.0
x_dot = 0.0
theta = 0.0
theta_dot = 0.5
states = np.array([[x], [x_dot], [theta], [theta_dot]])

# Placeholders for integration
states_l = np.copy(states)

# Sample times and sim length
t_final = 10

dt_plant = 0.001
dt_control = 0.01

"""   PLANT REPRESENTATION   """

# Plant SS Representation

P = I * (M_Cart + M_Arm) + (M_Cart * M_Arm * length**2)

A22 = (-(I + M_Arm * (length**2)) * b) / P
A23 = ((M_Arm**2) * g * (length**2)) / P
A42 = (-(M_Arm * length * b)) / P
A43 = (M_Arm * g * length * (M_Cart + M_Arm)) / P

B2 = (I + M_Arm * (length**2)) / P
B4 = (M_Arm * length) / P


# TODO
A = np.array(...)

B = np.array(...)

C = np.array(...)

D = np.array([[0], [0]])


inverse_pendulum_plant = signal.StateSpace(A, B, C, D)

# Convert to discrete time
inverse_pendulum_plant_d = inverse_pendulum_plant.to_discrete(dt_plant)

# Extract discrete state matrices
A_discrete = inverse_pendulum_plant_d.A
B_discrete = inverse_pendulum_plant_d.B
C_discrete = inverse_pendulum_plant_d.C
D_discrete = inverse_pendulum_plant_d.D


"""   CONTROLLER REPRESENTATION   """

# Initialize controller
if control_scheme == 1:  # PID Controller
    controller = PID(K_P, K_I, K_D, dt=dt_control, antiwindupmode=1, control_limit=1)

    # Eliminate spike in control force from start
    controller.last_error = -states_l[2][0]

elif control_scheme == 2:  # LQR Controller
    K, S, E = control.lqr(A, B, Q, R)
    # K[0,0] = 0 #no constraint on x
    # K[0,1] = 0 #no constraint on x_dot
    controller = LQR(K, max_input=20, max_input_on=False)

"""   OBSERVER REPRESENTATION   """

# Convert to discrete time
inverse_pendulum_plant_d_kalman = inverse_pendulum_plant.to_discrete(dt_control)

# Extract discrete state matrices
A_discrete_k = inverse_pendulum_plant_d_kalman.A
B_discrete_k = inverse_pendulum_plant_d_kalman.B
C_discrete_k = inverse_pendulum_plant_d_kalman.C
D_discrete_k = inverse_pendulum_plant_d_kalman.D

# print("A:", A)
print("A_d:", inverse_pendulum_plant_d.A)

# Kalman Filter

kf = KalmanFilter(...) # TODO

# initilize states and covariance
kf.x_last = states
kf.P_last = np.eye(np.size(A, 1)) * noise_val**2

# kf = kalman_filter(A, B, C, Q, R)

"""   UTILITY FUNCTIONS   """

# Take a state input and add random noise
def add_noise(states):

    for i in range(len(states - 1)):
        noise = (random.random() * 2 - 1) * noise_val
        states[i] += noise

    return states


"""   SIMULATION   """

# Time Vector
t_plant = np.arange(0, t_final, dt_plant)
t_control = np.arange(0, t_final, dt_control)

# Collector for states
states_coll = [[], [], [], []]  # real states
states_coll_n = [[], [], [], []]  # states w/ noise

measurement_error = [[], [], [], []]

# Collector for control force
control_force_coll = []

control_force = 0.0

full_step = int(dt_control / dt_plant)
steps = math.ceil(t_final / dt_control)

for i in range(0, steps):

    # If using noise, add in noise
    if measure_noise == 1:
        measurement = add_noise(states_l)

        # for for KF, run update to fill in states
        if observer == 1:
            # z = np.array([[measurement[0, 0]], [measurement[2, 0]]]) ## This was wrong in the source code
            measurement = kf.update(control_force, measurement)

    # for no noise, just pass states through
    else:
        measurement = states_l

    control_force = controller.update(measurement)

    # print("u:", control_force)
    # print("B:", B_discrete)

    control_force_coll = np.append(control_force_coll, control_force)

    states_coll_n = np.append(states_coll_n, measurement, axis=1)

    measurement_error = np.append(measurement_error, states_l - measurement, axis=1)

    for i in range(0, full_step):
        # Update states with ss eqs
        states = np.matmul(A_discrete, states_l) + B_discrete * control_force

        # Collect variables to plot
        states_coll = np.append(states_coll, states_l, axis=1)

        # Store info for next iteration
        states_l = states

# for i in t_plant:

#     if control_scheme == 1:   #PID Controller
#         # Calculate control action
#         control_force = controller.update(-states_l[2][0])
#     elif control_scheme == 2: #LQR Controller
#         control_force = controller.calc_force(states_l)

#         control_force_coll = np.append(control_force_coll, control_force)

#     # Update states with ss eqs
#     states = np.matmul(A_discrete, states_l) + B_discrete*control_force

#     # Collect variables to plot
#     states_coll = np.append(states_coll, states_l,axis=1)

#     # Store info for next iteration
#     states_l = states


"""   PLOTS   """

# Subplot 1 = states
# Subplot 2 = control force and error
fig, axs = plt.subplots(3)

axs[0].plot(t_plant, states_coll[:][0], label="x")
axs[0].plot(t_plant, states_coll[:][1], label="x_dot")
axs[0].plot(t_plant, states_coll[:][2], label="theta")
axs[0].plot(t_plant, states_coll[:][3], label="theta_dot")

axs[0].legend(loc="best", shadow=True, framealpha=1)

axs[1].plot(t_control, control_force_coll, label="Control Force")
axs[1].plot(t_plant, -states_coll[:][2], label="Error")

axs[1].legend(loc="best", shadow=True, framealpha=1)

# axs[2].plot(t_plant, states_coll[:][0], label='x')
# axs[2].plot(t_control, states_coll_n[:][0], label='x measurement')

# axs[2].plot(t_plant, states_coll[:][2], label='theta')
# axs[2].plot(t_control, states_coll_n[:][2], label='theta measurement')

axs[2].plot(t_control, measurement_error[:][2])

axs[2].legend(loc="best", shadow=True, framealpha=1)

plt.show()
