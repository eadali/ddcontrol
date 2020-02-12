#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan  9 20:18:58 2020

@author: eadali
"""
from time import time, sleep
from warnings import warn
from threading import Thread, Event



class PIDController(Thread):
    """Advenced PID controller interface.
    
    Args:
        kp (float): Proportional gain of controller
        ki (float): Integral gain of controller
        kd (float): Derivative gain of controller
        kn (float): Filter coefficient of controller
        freq(float, optional): PID controller calculation frequency
        lmin, lmax (float, optional): PID controller output limits
        
        
    Example:
        >>> from ddcontrol.control import PIDController
        >>> from ddcontrol.model import TransferFunction
        >>> import numpy as np
        >>> from matplotlib import pyplot as plt
        >>> import time
        
        >>> #Creates and starts PID controller
        >>> pid = PIDController(kp=30, ki=70.0, kd=1.0, kn=1.0)
        >>> pid.start()
        
        >>> #Creates transfer function model
        >>> mdl = TransferFunction([1.0], [1.0,10.0,20.0])
        >>> y, u = np.zeros(900, 'float32'), 0.0
        
        >>> #Control loop
        >>> start = time.time()
        >>> for index in range(y.size):
        >>>     t = time.time() - start
        >>>     y[index] = mdl.step(t, u)
        >>>     u = pid.update(1-y[index])
        >>>     time.sleep(0.001)
        
        >>> #Stops PID controller
        >>> pid.stop()
        >>> pid.join()
        
        >>> #Plots model output
        >>> plt.plot(y)
        >>> plt.show()
    """    
    def __init__(self, kp, ki, kd, kn, freq=10.0, lmin=-float('Inf'), lmax=float('Inf')):
        Thread.__init__(self)
        #Stop event for thread
        self.sflag = Event()
        # Gains
        self.kp, self.ki, self.kd, self.kn = kp, ki, kd, kn
        #Other parameters
        self.freq, self.lmin, self.lmax = freq, lmin, lmax
        self.reset()
        
        
    def __sign(self, x):
        if x < 0.0:
            return -1.0
        elif x > 0.0:
            return 1.0
        return 0.0
    
    
    def __clip(self, x, lmin, lmax):
        if x < lmin:
            return lmin
        elif x > lmax:
            return lmax
        return x
    
    
    def __isclose(self, x1, x2, tol=1e-15):
        return abs(x1-x2) < tol
        

    def step(self, e, dt):
        """Calculates PID controller states and returns output of controller
        
        Args:
            e (float): Input value of controller
            dt (float): Time step used for integral calculations
        """
        #Calculates proportional term
        up = self.kp * e
        # Calculates integral term
        if not self.windup:
            self.int += self.ki * e * dt
        ui = self.int
        # Calculates derivative term
        ud = self.kn * (self.kd * e - self.fint)
        self.fint += ud * dt
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

    
    def reset(self):
        """Resets PID controller state.
        """
        #Integral value of integral term and derivative term
        self.int, self.fint = 0.0, 0.0
        #Error and control signal value
        self.e, self.u = 0.0, 0.0
        #Previously measured timestamp
        self.ptime = None
        # Anti-Wind Up status
        self.windup = False
    
    
    def run(self):
        #Loop for thread
        self.sflag.clear()
        while not self.sflag.is_set():
            #Fixes frequency of controller
            if self.ptime is not None:
                wait = (1.0/self.freq) - (time()-self.ptime)
                if wait > 0.0:
                    sleep(wait)
                else:
                    warn('PID contoller frequency is lower than \'freq\' value.', RuntimeWarning)
            self.ptime = time()
            self.step(self.e, (1.0/self.freq))


    def stop(self):
        """Stops PID controller thread
        """
        warn('bele vaziyyetin icine soxum.', RuntimeWarning)
        self.sflag.set()
        



from scipy.optimize import minimize
from numpy import  zeros, arange, inf, isnan, diff, exp, square
   


def pidopt(tf, end=10.0, wd=0.5, k0=(0.0,0.0,0.0,0.0), freq=10.0, lim=(-float('Inf'),float('Inf'))):
    """PID optimization function for given transfer function
    Args:
        tf (TransferFunction): TransferFunction object for optimization.
        end (float, optional): Optimization end time
        wd (float, optional): Disturbance loss weight [0,1].
        k0 (tuple, optional): Initial PID controller gains.
        freq (float, optional): PID controller frequency.
        lim (tuple, optional): Output limit values of PID controller
    
    Todo:
        Optimization is very slow. Improve performance
    """
    #Creates timestamps and time ranges
    dt = 1.0/freq
    t = arange(0, 2*end, dt)
    #Creates input, output and disturbance arrays
    y, u, d = zeros(t.size, 'float32'), zeros(t.size, 'float32'), zeros(t.size, 'float32')
    split = int(0.5 * t.size)
    srange = int(t.size * 0.05)
    d[split:] = 1.0
    #Objective function for pid gains
    pid = PIDController(kp=k0[0], ki=k0[1], kd=k0[2], kn=k0[3], freq=freq, lmin=lim[0], lmax=lim[1])
    def objective(k):
        #Initializes controller
        pid.kp, pid.ki, pid.kd, pid.kn = k
        tf.set_initial_value()
        pid.set_initial_value()
        #Control loop
        for index in range(1, t.size):
            y[index] = tf.step(t[index], u[index-1]+d[index])
            u[index] = pid.step(dt, 1.0-y[index])
        #If the signals contains nan, loss is infinite
        if isnan(y).any() or isnan(u).any():
            return inf
        #Calculates stability loss
        sloss = square(diff(y[split-srange:split])).mean()
        sloss += square(diff(y[split:split+srange])).mean()
        #Calculates tracking loss
        tloss = square(1.0-y[:split]).mean()
        #Calculates disturbance loss
        dloss = square(1.0-y[split:]).mean()
        loss = exp(sloss) + 100*(1-wd)*tloss + 100*wd*dloss
        return loss
    #Optimize pid gains
    res = minimize(objective, x0=(pid.kp, pid.ki, pid.kd, pid.kn),
                   method='SLSQP', options={'maxiter':40,'eps':1e-2})
    tf.set_initial_value()
    pid.kp, pid.ki, pid.kd, pid.kn = res.x
    pid.reset()
    return pid, res