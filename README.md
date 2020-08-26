# Inverted Pendulum

https://github.com/jpaine126/Inverted_Pendulum_Control_Demo

The model used for this project is taken from the "Controls Tutorials for Matlab and Simulink" course on inverted pendulums, found [here](http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling). All images shown are also taken from this site.

## Overview

This section will give a brief overview about the Inverted Pendulum Dynamic System, and will discuss the different control actions and options in the simulation.

### The Inverted Pendulum

The inverted pendulum is a classic example of an unstable dynamic system. The goal of this project is to simulate this system, and to design and implement a control scheme that balances the pendulum. More info on the dynamics can be found [here](http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling).

![Figure 1](http://ctms.engin.umich.edu/CTMS/Content/InvertedPendulum/System/Modeling/figures/pendulum.png "Inverted Pendulum Model")

## Running the Simulation

Run the simulation by running the main file inverted_pendulum.py. Listed in the next few sections are the different parameters that can be changed in the simulation. After the simulation is done running, the states and the control force/error are plotted.

### Files and Libraries

```
Included Files:
inverted_pendulum.py
PID.py
LQR.py
```

```
Libraries:
Numpy
Scipy
Python Control Systems Library
Matlibplot
```
### Physical Paramters

```
M_Cart  = 0.5  % kg
M_Arm   = 0.2  % kg
length  = 0.3  % m
b       = 0.1  % N/m/s
g       = 9.8  % m/s^2
```

#### Controller Parameters
##### PID Controller

The PID Controller has three gains for the proportional, integral, and derivative of the error signal. 

```
K_P = 10    % Proportional Gain
K_I = 1     % Integral Gain
K_D = 1     % Derivative Gain
```
##### Linear Quadratic Regulator

The LQR has three parameters that can be tuned to change the controller response. q11 and q33 represent the relative cost of the associated state in the system, and r represents the cost of the control force, meaning that a higher value causes the controller to react more strongly to that state. Individual gains can be manually set to zero after calculation to see the effect of eliminating the cost of that state.

```
q11 = 5000  % Cost on position error
q33 = 100   % Cost on angle error

r   = 1     % Cost on control force
```
#### Initial Conditions
```
x         = 0  % Initial Cart Position - m
x_dot     = 0  % Initial Cart Velocity - m/s
theta     = 0  % Initial Pendulum Angle - rad
theta_dot = 2  % Initial Pendulum Angular V - rad/s
```

#### Time and Simulation Parameters
```
t_final       = 10     % Length of Simulation - s
dt_plant      = 0.001  % Size of Time Step for Plant Dynamics - s
dt_control    = 0.01   % Size of Time Step for Controller Update - s
```

## Planned Features

- [x] Multiple Control Algorithms, including Linear Quadratic Regulator.
- [ ] Adding noise and multiple types of observers to the system.
- [ ] A graphical interface in QT that will allow the user to select the control algorithm from a drop down menu, and will have text input for changing gains, parameters, sample time, and initial conditions.
