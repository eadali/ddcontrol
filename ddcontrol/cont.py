#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan  9 20:18:58 2020

@author: eadali
"""
from numpy import clip, inf, sign, absolute
from time import time, sleep
from warnings import warn


class PIDController:
    def __init__(self, kp, ki, kd, kn, freq=10.0, lmin=-inf, lmax=+inf):
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
        self.kn = kn
        #Other parameters
        self.freq = freq
        self.lmin = lmin
        self.lmax = lmax

        #Integral value of PID
        self.integral = 0.0
        #Integral value of filter
        self.filt_integral = 0.0
        #Previously measured time
        self.past = None
        # Anti-Wind Up status
        self.windup = False


    def update(self, err):
        #Fixes frequency of controller
        if self.past is not None:
            wait = (1.0/self.freq) - (time()-self.past)
            if wait > 0.0:
                sleep(wait)
            else:
                warn('PID contoller frequency is lower than \'freq\' value', RuntimeWarning)
        self.past = time()

        #Calculates proportional term
        u_proportional = self.kp * err
        # Calculates integral term
        if not self.windup:
            self.integral += self.ki * err * (1.0/self.freq)
        u_integral = self.integral
        # Calculates derivative term
        u_derivative = self.kn * (self.kd * err - self.filt_integral)
        self.filt_integral += u_derivative * (1/self.freq)
        #Anti-windup
        u_sum = u_proportional + u_integral + u_derivative
        self.windup = False
        if u_sum < self.lmin or u_sum > self.lmax:
            if absolute(sign(err) - sign(u_sum)) < 1e-15:
                self.windup = True
        # Calculates output
        u_control =  clip(u_sum, self.lmin, self.lmax)
        return u_control