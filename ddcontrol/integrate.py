#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  6 22:05:52 2020

@author: eadali
"""
from scipy.interpolate import interp1d
from collections import deque
from numpy import array, isscalar



class ODE:
    """
    A generic interface class to numeric integrators.
    Solve an equation system :math:`y'(t) = f(t,y)`.
    Parameters
    ----------
    f : callable ``f(t, y, *f_args)``
        Right-hand side of the differential equation. t is a scalar,
        ``y.shape == (n,)``.
        ``f_args`` is set by calling ``set_f_params(*args)``.
        `f` should return a scalar, array or list (not a tuple).
    Attributes
    ----------
    t : float
        Current time.
    y : ndarray
        Current variable values.
    """
    def __init__(self, f):
        self.f = f


    def __asarray(self, x):
        if isscalar(x):
            x = [x]
        return array(x)


    def set_initial_value(self, t, y):
        """Set initial conditions y(t) = y."""
        self.t = t
        self.y = self.__asarray(y)
        return self


    def set_f_params(self, *args):
        """Set extra parameters for user-supplied function f."""
        self.f_params = args
        return self


    def integrate(self, t):
        """Find y=y(t), set y as an initial condition, and return y.
        Parameters
        ----------
        t : float
            The endpoint of the integration step.
        Returns
        -------
        y : float
            The integrated value at t
        """        
        # Calculate step-size 
        h = t - self.t
        # Calculate slope at the beginning of the interval
        k1 = self.f(self.t, self.y, *self.f_params)
        k1 = self.__asarray(k1)
        # Calculate slope at the midpoint of the interval
        k2 = array(self.f(self.t + h/2, self.y + (h/2)*k1, *self.f_params))
        k2 = self.__asarray(k2)
        # Calculate slope at the midpoint with k2
        k3 = array(self.f(self.t + h/2, self.y + (h/2)*k2, *self.f_params))
        k3 = self.__asarray(k3)
        # Calculate slope at the end of the interval
        k4 = array(self.f(self.t + h/2, self.y + h*k3, *self.f_params))
        k4 = self.__asarray(k4)
        # Update time and state
        self.t = self.t + h
        self.y = self.y + (h/6) * (k1+ 2*k2 + 2*k3 + k4)
        return self.y



class DDE(ODE):
    """A interface to to numeric integrator for Delay Differential Equations.
    For more detail: Thanks to http://zulko.github.io/

    Args:
        f (callable): Right-hand side of the differential equation.
        jac (callable, optional): Jacobian of the right-hand side.
    """
    def __init__(self, f):
        w = lambda t, y, args: array(f(t, self.cint, *args), 'float32')
        ODE.__init__(self, w)
        self.set_f_params(())


    def set_initial_value(self, t, g):
        """Sets initial conditions

        Args:
            g (callable): A python function or method for t<t0.
            t0 (float, optional): Time value for condition
        """
        self.t = t
        w = lambda t: array(g(t))
        self.cint = CInterp1d(w, t)
        ODE.set_initial_value(self, t, w(t))


    def integrate(self, t):
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
        if t < self.t:
            y = array(self.cint(t))
        else:
            y = array(ODE.integrate(self, t), 'float32')
            self.cint.append(t, y)
        return y



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



