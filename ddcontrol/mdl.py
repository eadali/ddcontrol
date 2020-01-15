# -*- coding: utf-8 -*-
"""
Created on Wed Jan 15 10:06:42 2020

@author: ERADALI
"""
from time import time
from scipy.integrate import odeint



class MSDModel:
    def __init__(self, m, k, b, x_0):
        """Inits mass-spring-damper model
        """
        self.m = m
        self.k = k
        self.b = b
        self.x_0 = x_0
        self.past = None


    def ode(self, x, t, u):
        """Dynamic equations of mass-spring-damper
        """
        # ODE of pendulum
        pos, vel = x
        dxdt = [vel, -(self.k/self.m)*pos -(self.b/self.m)*vel +(1.0/self.m)*u]
        return dxdt


    def update(self, u):
        """Interface function for mass-spring model
        """
        now = time()
        dt = 0.0
        if self.past is not None:
            dt = now - self.past
        self.past = now
        x = odeint(self.ode, self.x_0, [0.0, dt], args=(u,))
        self.x_0 = x[1,:]
        return x[1,0]