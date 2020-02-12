#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  6 22:05:52 2020

@author: eadali
"""
from scipy.interpolate import interp1d
from collections import deque
from scipy.integrate import ode
from numpy import array



class CInterp1d:
    """A conditional interpolation function interface.

    This class returns a value defined as y=g(x) if x<x0, else interpolate(x)

    Args:
        g (callable, g(x)): A python function or method for x<x0.
        x0 (float, optional): Condition value for x.
        lsize (int, optional): Limit size for interpolate history
    """
    def __init__(self, g, x0=0, lsize=1e4):
        self.g = g
        self.x0 = x0
        self.lsize = lsize
        self.x = deque([self.x0-2e-9, self.x0-1e-9])
        self.y = deque([array(self.g(self.x0-2e-9), 'float32'),
                        array(self.g(self.x0-1e-9), 'float32')])
        self.interp1d = interp1d(self.x, self.y, axis=0, fill_value='extrapolate')


    def __call__(self, x):
        """Returns a value defined as y=g(x) if x<x0, else interpolate(x)
            
        Args: 
            x (float): Input value
        
        Returns:
            float: Conditional interpolate value
        """
        if x < self.x0:
            return array(self.g(x), 'float32')
        else:
            return array(self.interp1d(x), 'float32')


    def append(self, x_new, y_new):
        """Appends new values to interpolation

        Args:
            x_new (float): New x value for interpolation
            y_new (float): New y value for interpolation
        """
        self.x.append(x_new)
        self.y.append(array(y_new, 'float32'))
        if len(self.x) > self.lsize:
            self.x.popleft()
            self.y.popleft()
        self.interp1d = interp1d(self.x, self.y, axis=0, fill_value='extrapolate')



class DDE(ode):
    """A interface to to numeric integrator for Delay Differential Equations.
    For more detail: Thanks to http://zulko.github.io/
    
    Args:
        f (callable): Right-hand side of the differential equation.
        jac (callable, optional): Jacobian of the right-hand side.
    """
    def __init__(self, f, jac=None):
        w = lambda t, y, args: array(f(t, self.cint, *args), 'float32')
        ode.__init__(self, w, jac)
        self.set_f_params(())


    def set_initial_value(self, g, t0=0.0):
        """Sets initial conditions 
        
        Args:
            g (callable): A python function or method for t<t0.
            t0 (float, optional): Time value for condition
        """
        self.t0 = t0
        w = lambda t: array(g(t))
        self.cint = CInterp1d(w, t0)
        ode.set_initial_value(self, w(t0), t0)


    def integrate(self, t, step=False, relax=False):
        """Find y=y(t), set y as an initial condition, and return y.
        
        Args:
            t (float): The endpoint of the integration step.
            step (bool): If True, and if the integrator supports the step method,
            then perform a single integration step and return.
            This parameter is provided in order to expose internals of
            the implementation, and should not be changed from its default
            value in most cases.
            relax (bool): If True and if the integrator supports the run_relax method,
            then integrate until t_1 >= t and return. ``relax`` is not
            referenced if ``step=True``.
            This parameter is provided in order to expose internals of
            the implementation, and should not be changed from its default
            value in most cases.
        
        Returns:
            float: The integrated value at t.
        """
        if t < self.t0:
            y = array(self.cint(t))
        else:
            y = array(ode.integrate(self, t, step, relax), 'float32')
            self.cint.append(t, y)
        return y
