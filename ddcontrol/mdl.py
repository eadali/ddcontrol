# -*- coding: utf-8 -*-
"""
Created on Wed Jan 15 10:06:42 2020

@author: ERADALI
"""
from time import time
from scipy.integrate import odeint



class MSDModel:
    def __init__(self, m, k, b, x0):
        """Inits mass-spring-damper model
        #Arguments:
            m: Mass value
            k: Spring value
            b: Damper value
            x0: Initial states
        """
        #System Parameters
        self.m, self.k, self.b = m, k, b
        #Initial states
        self.x0 = x0
        #Previously measured timestamp
        self.past = None


    def ode(self, x, t, u):
        """Dynamic equations of mass-spring-damper
        #Arguments:
            x: States of ode
            t: Timestamps
            u: Control signal value
        #Returns
            Derivative of states
        """
        # ODE of pendulum
        pos, vel = x
        dxdt = [vel, -(self.k/self.m)*pos - (self.b/self.m)*vel + (1.0/self.m)*u]
        return dxdt


    def update(self, u):
        """Interface function for mass-spring-damper model
        #Arguments:
            u: Control signal value
        #Returns:
            Position of mass
        """
        #Calculates output signal
        now = time()
        dt = 0.0
        if self.past is not None:
            dt = now - self.past
        self.past = now
        x = odeint(self.ode, self.x0, [0.0, dt], args=(u,))
        self.x0 = x[1,:]
        return x[1,0]