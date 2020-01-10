#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan  9 20:18:58 2020

@author: eadali
"""
from numpy import clip, inf
from time import time
from scipy.optimize import minimize



class Integral:
    def __init__(self, initial=0.0):
        """Inits Integral parameters
        # Arguments
            initial: Initial state of integral
        """
        # Initial state of integral
        self.state = initial
        # Previously measured time
        self.ttime_prev = None


    def update(self, u_in):
        """Interface function for Integral
        # Arguments
            u_in: Input value
        # Returns
            The state of integral
        """
        ttime = time()
        if self.ttime_prev is not None:
            self.state += (ttime-self.ttime_prev)*u_in
        self.ttime_prev = ttime
        return self.state



class PIDController:
    def __init__(self, kp, ki, kd, nn, lmin=-inf, lmax=+inf):
        """Inits PIDContoller parameters
        # Arguments
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        # Gains
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.nn = nn
        self.lmin = lmin
        self.lmax = lmax

        # Integral value of PID
        self.int = Integral()
        # Integral value of filter
        self.filt_int = Integral()
        # Anti-Wind Up status
        self.anti_wind = False


    def update(self, err):
        """Interface function for PIDController
        # Arguments
            err: error value
        # Returns
            control signal for model
        """
        # Calculates proportional term
        u_prop = self.kp*err
        # Calculates integral term
        ##TODO: implement wind up
        if self.anti_wind:
            u_int = self.int.update(0.0)
        else:
            u_int = self.int.update(self.ki*err)
        # Calculates derivative term
        u_deriv = self.nn * (self.kd*err-self.filt_int.state)
        self.filt_int.update(u_deriv)
        # Calculates output
        u_sum = u_prop + u_int + u_deriv
        u_control = clip(u_sum, self.lmin, self.lmax)
        return u_control


#class PIDTuner:
#    def __init__(self, controller, tbandwidth, tphasemargin, ptype, psign, amp):
#        self.controller = controller
#        self.tbandwidth = tbandwidth
#        self.tphasemargin = tphasemargin
#        self.ptype = ptype
#        self.psign = psign
#
#
#    def update(self, err, y_meas, tune=False):
#        angle = asarray([1.0/16.0, 1.0/4.0, 1.0, 4.0, 16.0]) * self.tbandwidth
#        disturb = amp * sin(angle).sum()
#
#
#        angle[1/16]
#        u_control = self.controller.update(err)
#        return u_control


#class FFTResponse:
#    def __init__(self, fmin, fmax, num):
#        freq = linspace(fmin, fmax, num)
#        # Previously measured time
#        self.ttime_start = None
#
#    def update(self, y_meas, u_cont):
#        ttime = 0.0
#        if self.ttime_start is not None:
#            ttime = time() - self.ttime_start
#
#        u_control = sin(freq*)






