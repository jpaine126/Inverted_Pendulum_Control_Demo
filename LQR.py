#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 25 17:28:04 2020

@author: jpaine

LQR Controller Class

This class if for implementing a LQR controller, not for designing one.



"""


class LQR:

    def __init__(self, K):
        self.K = K
        
    def set_gain(self, K):
        self.K = K
        
        
    def calc_force(self, states):
      
        control = 0
        
        for i in range(0, len(states)):
            control += states[i,0]*self.K[0,i]
            #print(states[i,0])
            #print(self.K[0,1])
            
        return -control