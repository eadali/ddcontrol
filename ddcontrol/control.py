#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan  9 20:18:58 2020

@author: eadali
"""
from time import time, sleep
from warnings import warn
from threading import Thread, Event
from numpy import inf



class PIDController:
    """Advenced PID controller interface.

    Args:
        kp (float): Proportional gain of controller.
        ki (float): Integral gain of controller.
        kd (float): Derivative gain of controller.
        kn (float): Filter coefficient of controller.
        freq(float, optional): PID controller calculation frequency.
        lmin, lmax (float, optional): PID controller output limits.


    Example:
        >>> from ddcontrol.model import TransferFunction
        >>> from ddcontrol.control import PIDController
        >>> import numpy as np
        >>> import matplotlib.pyplot as plt
        >>> import time

        >>> #Creates PID controller and test model
        >>> tf = TransferFunction([1.0], [1.0,10.0,20.0])
        >>> pid = PIDController(kp=30, ki=70.0, kd=0.0, kn=0.0)
        >>> ref = 1.0

        >>> #Control loop
        >>> pid.start()
        >>> y, u = np.zeros(900), 0.0
        >>> start = time.time()
        >>> for index in range(y.size):
        >>>     t = time.time() - start
        >>>     y[index] = tf.step(t, u)
        >>>     u = pid.update(ref-y[index])
        >>>     time.sleep(0.001)

        >>> #Stops PID controller
        >>> .stop()
        >>> pid.join()

        >>> #Plots result
        >>> fig, ax = plt.subplots()
        >>> ax.plot(y)
        >>> ax.grid()
        >>> plt.show()
    """
    def __init__(self, Kp, Ki, Kd, Kn, output_limits=(-inf, inf)):
        # Get PID controller gains and output limits
        self.Kp, self.Ki, self.Kd, self.Kn = Kp, Ki, Kd, Kn
        self.output_limits = output_limits
    
    def set_initial_value(T0, I0, D0):
        self.t = T0
        
    def integrate(self, t, e):
        """Calculates PID controller states and returns output of controller

        Args:
            u (float): Input value of controller
            dt (float): Time step used for integral calculations
        """
        #Calculates proportional term
        up = self.kp * e
        # Calculates integral term
        if not self.windup:
            self.integ += self.ki * e * dt
        ui = self.integ
        # Calculates derivative term
        ud = self.kn * (self.kd * e - self.finteg)
        self.finteg += ud * dt
        #Anti-windup
        u = up + ui + ud
        self.windup = False
        if u < self.lmin or u > self.lmax:
            if self.__isclose(self.__sign(e),self.__sign(u)):
                self.windup = True
        # Calculates output
        self.u =  self.__clip(u, self.lmin, self.lmax)
        return self.u


    def update(self, e):
        """Updates error value of PIDController.

        Args:
            e (float): Error signal value

        Returns:
            float: Control signal value
        """
        self.e = e
        return self.u


    def set_initial_value(self, integ=0.0, finteg=0.0):
        """Resets PID controller state.
        """
        #Integral value of integral term and derivative term
        self.integ, self.finteg = integ, finteg
        #Error and control signal value
        self.e, self.u = 0.0, 0.0
        #Previously measured timestamp
        self.ptime = None
        # Anti-Wind Up status
        self.windup = False



from scipy.optimize import minimize
from numpy import  zeros, arange, inf, isnan, absolute



def pidopt(tf, end=10.0, wd=0.5, k0=(1.0,1.0,1.0,1.0), freq=10.0, lim=(-float('Inf'),float('Inf'))):
    """PID optimization function for given transfer function

    Args:
        tf (TransferFunction): TransferFunction object for optimization.
        end (float, optional): Optimization end time
        wd (float, optional): Disturbance loss weight [0,1].
        k0 (tuple, optional): Initial PID controller gains.
        freq (float, optional): PID controller frequency.
        lim (tuple, optional): Output limit values of PID controller

    Returns:
        tuple: Optimized PIDController and OptimizeResult.

    Example:
        >>> from ddcontrol.model import TransferFunction
        >>> from ddcontrol.control import pidopt

        >>> #Creates transfer function
        >>> tf = TransferFunction([1.0], [1.0,10.0,20.0], udelay=0.1)

        >>> #Optimizes pid controller
        >>> pid, _ = pidopt(tf)
        >>> print('Optimized PID gains..:', pid.kp, pid.ki, pid.kd, pid.kn)

    Todo:
        Optimization is very slow. Improve performance
    """
    #Creates timestamps and time ranges
    dt = 1.0 / freq
    t = arange(0, end, dt)
    #Creates input, output and disturbance arrays
    y, u = zeros(t.size, 'float32'), zeros(t.size, 'float32')
    #Objective function for pid gains
    pid = PIDController(kp=k0[0], ki=k0[1], kd=k0[2], kn=k0[3], freq=freq, lmin=lim[0], lmax=lim[1])
    def objective(k):
        #Initializes controller
        pid.kp, pid.ki, pid.kd, pid.kn = k
        tf.set_initial_value()
        pid.set_initial_value()
        #Control loop
        for index in range(1, t.size):
            y[index] = tf.step(t[index], u[index-1]+wd)
            u[index] = pid.step(dt, 1.0-y[index])
        #If the signals contains nan, loss is infinite
        if isnan(y).any() or isnan(u).any():
            return inf
        #Calculates loss
        loss = (t * absolute(1.0-y)).mean()
        return loss
    #Optimize pid gains
    res = minimize(objective, x0=(pid.kp, pid.ki, pid.kd, pid.kn),
                   method='SLSQP', options={'ftol':1e-4, 'eps':1e-2})
    tf.set_initial_value()
    pid.kp, pid.ki, pid.kd, pid.kn = res.x
    pid.set_initial_value()
    return pid, res