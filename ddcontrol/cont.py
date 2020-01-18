#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan  9 20:18:58 2020

@author: eadali
"""
from numpy import clip, inf, sign, absolute, sin, pi, asarray, nan, full, roll
from numpy import isnan
from scipy.signal import lsim
from scipy.optimize import curve_fit
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



class PIDTuner:
    def __init__(self, controller, freq, amplitude):
        self.controller = controller
        self.freq = freq * asarray([1.0/10.0, 1.0/3.0, 1.0, 3.0, 10.0])
        self.amplitude = amplitude
        self.start = None
        self.gains = None
        self.history = full((3,int(200.0/self.freq[2])), nan, 'float32')


    def model(u, b0, b1, b2, b3, a1, a2, a3, x01, x02, x03):
        timestamps = u[:,0]
        u_cont = u[:,1]
        num = [b0, b1, b2, b3]
        den = [1.0, a1, a2, a3]
        x0 = [x01, x02, x03]
        _, y, _ = lsim((num,den), U=u_cont, T=timestamps, X0=x0)
        return y


    def fit_model(self):
        self.gains = curve_fit(self.model, self.history[:,(0,1)], self.history[:,2])


    def update(self, err, y_meas, tune=False):
        u_control = self.controller.update(err)
        disturbance = 0.0
        if tune:
            now = time()
            if self.start is None:
                self.start = now
            ctime = now - self.start
            disturbance = 0.1 * sin(2.0*pi*self.freq*ctime).sum()
            self.history = roll(self.history, -1, axis=0)
            self.history[-1,0] = ctime
            self.history[-1,1] = u_control
            self.history[-2,2] = y_meas
#            if not isnan(self.history).any():
#                self.fit_model()
#                self.history = full((3,int(200.0/self.freq[2])), nan, 'float32')
#        else:
#            self.history = full((3,int(200.0/self.freq[2])), nan, 'float32')

        return u_control+disturbance