#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 24 13:36:32 2020

@author: jpaine
"""

# Libraries
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

# custom classes
from PID import PID

"""   SIMULATION PARAMETERS   """

# Physical Parameters

M_Cart  = 0.5
M_Arm   = 0.2
length  = 0.3
b       = 0.1
g       = 9.8
I       = (1/3)*M_Arm*(length**2)

# Controller Parameters

K_P = 50
K_I = 1
K_D = 1

# Initialize States
x         = 0
x_dot     = 0
theta     = .1   # Vertical at theta=pi rads
theta_dot = 0
states = np.array([[x], [x_dot], [theta], [theta_dot]])

# Placeholders for integration
states_l = np.copy(states)

# Sample times and sim length
t_final       = 10

dt_plant      = 0.001
dt_controller = 0.01

"""   PLANT REPRESENTATION   """

# Plant SS Representation

P = ( I*(M_Cart+M_Arm) + (M_Cart*M_Arm*length**2) )

A22 = ( -(I+M_Arm*(length**2))*b ) /  P
A23 = ( (M_Arm**2)*g*(length**2) ) / P
A42 = ( -(M_Arm*length*b) ) / P
A43 = ( M_Arm*g*length*(M_Cart+M_Arm) ) / P

B2 = ( I + M_Arm*(length**2) ) / P
B4 = ( M_Arm*length ) / P


A = np.array([[0, 1, 0, 0],
              [0, A22, A23, 0],
              [0, 0, 0, 1],
              [0, A42, A43, 0]])

B = np.array([[0], [B2], [0], [B4]])

C = np.array([[1, 0, 0, 0],
              [0, 0, 1, 0]])

D = np.array([[0],[0]])


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
controller = PID(K_P, K_I, K_D, dt=dt_plant, antiwindupmode=1)
controller.set_control_limit(1)

"""   SIMULATION   """

# Time Vector
t = np.arange(0, t_final, dt_plant)

# Collector for states
states_coll = [[],[],[],[]]

# Collector for control force
control_force_coll = []

# Eliminate spike in control force from start
controller.set_last_error(-states_l[2][0])

for i in t:
    # Calculate control action
    control_force = controller.update(-states_l[2][0])
    
    # Update states with ss eqs
    states = np.matmul(A_discrete, states_l) + B_discrete*control_force
    
    # Collect variables to plot
    states_coll = np.append(states_coll, states_l,axis=1)
    control_force_coll = np.append(control_force_coll, control_force)
    
    # Store info for next iteration
    states_l = states


"""   PLOTS   """

# Subplot 1 = states
# Subplot 2 = control force and error
fig, axs = plt.subplots(2)

axs[0].plot(t, states_coll[:][0], label='x')
axs[0].plot(t, states_coll[:][1], label='x_dot')
axs[0].plot(t, states_coll[:][2], label='theta')
axs[0].plot(t, states_coll[:][3], label='theta_dot')

axs[0].legend(loc='best', shadow=True, framealpha=1)

axs[1].plot(t, control_force_coll, label = 'Control Force')
axs[1].plot(t, -states_coll[:][2], label = 'Error')

axs[1].legend(loc='best', shadow=True, framealpha=1)

plt.show()
    
    














