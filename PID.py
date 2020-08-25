#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 24 14:03:15 2020

@author: jpaine

PID Controller Class



Antiwideup Modes:
    0 - No Antiwindup Scheme
    
    1 - Saturate the Integrator Variable


"""

from math import copysign


class PID:
    

    def __init__(self, K_P, K_I, K_D, dt=1, antiwindupmode=0):
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.dt  = dt
        self.last_error = 0
        self.integrator = 0
        self.control_limit = 10
        self.antiwindup_mode = antiwindupmode
        
    " Getters and Setters "
        
    def set_kp(self, K_P):
        self.K_P = K_P 
    def set_ki(self, K_I):
        self.K_I = K_I
    def set_kd(self, K_D):
        self.K_D = K_D
        
    def set_dt(self, dt):
        self.dt = dt
        
    def set_last_error(self, last_error):
        self.last_error = last_error
        
    def set_integrator(self, integrator):
        self.integrator = integrator
        
    def set_control_limit(self, control_limit):
        self.control_limit = control_limit
        
    def set_antiwindup_mode(self, antiwindup_mode):
        self.antiwindup_mode = antiwindup_mode
        
    " Utility Functions "
    

    
    " Control Force Update Function "
        
    def update(self, error):
        integral = self.integrator + error*self.dt
        deriv = (error - self.last_error)/self.dt
        
        if self.antiwindup_mode == 0:
            control = error*self.K_P + integral*self.K_I + deriv*self.K_D
        
        elif self.antiwindup_mode == 1:
            if abs(integral) > self.control_limit:
                integral = copysign(self.control_limit, integral)
            control = error*self.K_P + integral*self.K_I + deriv*self.K_D
           
        self.last_error = error
        self.integrator = integral
        
        return control
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    