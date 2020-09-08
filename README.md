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

On line 124 there are a couple other parameters that can be changed pertaining to the antiwind up  on the integrator term (`antiwindupmode, control_limit`). This shouldn't need to be changed at all, but you can experiment with making the control action weaker with the antiwindup turned off, to see why it is necessary in the first place.


##### Linear Quadratic Regulator

The LQR has three parameters that can be tuned to change the controller response. q11 and q33 represent the relative cost of the associated state in the system, and r represents the cost of the control force, meaning that a higher value causes the controller to react more strongly to that state. Individual gains can be manually set to zero after calculation to see the effect of eliminating the cost of that state.

```
q11 = 5000  % Cost on position error
q33 = 100   % Cost on angle error

r   = 1     % Cost on control force
```

On line 138, there are two parameters for setting a maximum control force for the LQR (`max_input, max_input_on`). By default it is turned off. This may need to be turned on when using a lot of noise to ensure a reasonable response.

#### Initial Conditions
```
x         = 0  % Initial Cart Position - m
x_dot     = 0  % Initial Cart Velocity - m/s
theta     = 0  % Initial Pendulum Angle - rad
theta_dot = 2  % Initial Pendulum Angular V - rad/s
```

#### Time and Simulation Parameters
```
# Selected Control Scheme
control_scheme = 2
#  1 - PID
#  2 - LQR

# Noise
measure_noise = 0
#  0 - No Noise
#  1 - Noise

noise_val = 0.01

...

t_final       = 10     % Length of Simulation - s
dt_plant      = 0.001  % Size of Time Step for Plant Dynamics - s
dt_control    = 0.01   % Size of Time Step for Controller Update - s
```

The dt\_control should be chosen so that dt\_plant is a multiple of dt\_control. 

## Planned Features

- [x] Multiple Control Algorithms, including Linear Quadratic Regulator.
- [ ] Adding noise and multiple types of observers to the system.
- [ ] Changing the plant simulation to use the full non-linear equations of motion.
- [ ] A graphical interface in QT that will allow the user to select the control algorithm from a drop down menu, and will have text input for changing gains, parameters, sample time, and initial conditions.