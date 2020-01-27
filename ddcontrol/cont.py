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
    def __init__(self, kp, ki, kd, kn, freq=10.0, lmin=-float('Inf'), lmax=float('Inf')):
        """Inits PIDContoller parameters
        #Arguments
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        Thread.__init__(self)
        #Stop event for thread
        self.stop_event = Event()
        # Gains
        self.kp, self.ki, self.kd, self.kn = kp, ki, kd, kn
        #Other parameters
        self.freq, self.lmin, self.lmax = freq, lmin, lmax
        #Integral value of integral term
        #Integral value of derivative term
        self.integral, self.filt_integral = 0.0, 0.0
        #Error signal value
        #Control signal value
        self.err, self.u_control = 0.0, 0.0
        #Previously measured timestamp
        self.past = None
        # Anti-Wind Up status
        self.windup = False
    
    
    def sign(self,value):
        """Sign function
        #Arguments:
            value: value for sign detection
        #Returns:
            Sign of value
        """
        sgn = 0.0
        if value < 0.0:
            sgn = -1.0
        elif value > 0.0:
            sgn = 1.0
        return sgn


    def clip(self, value, lmin, lmax):
        """Clip function
        #Arguments:
            value: value for clip
            lmin: Minimum limit
            lmax: Maximum limit
        #Returns:
            Clipped value
        """
        clip_val = value
        if value > lmax:
            clip_val = lmax
        elif value < lmin:
            clip_val = lmin
        return clip_val


    def update(self, err):
        """Update function for PID controller
        #Arguments
            err: Error signal value
        #Returns
            Value of control signal
        """        
        self.err = err
        return self.u_control
    
    
    def run(self):
        """Loop for thread
        """
        self.stop_event.clear()
        while not self.stop_event.is_set():
            #Fixes frequency of controller
            if self.past is not None:
                wait = (1.0/self.freq) - (time()-self.past)
                if wait > 0.0:
                    sleep(wait)
                else:
                    warn('PID contoller frequency is lower than \'freq\' value', RuntimeWarning)
            self.past = time()
    
            #Calculates proportional term
            u_proportional = self.kp * self.err
            # Calculates integral term
            if not self.windup:
                self.integral += self.ki * self.err * (1.0/self.freq)
            u_integral = self.integral
            # Calculates derivative term
            u_derivative = self.kn * (self.kd * self.err - self.filt_integral)
            self.filt_integral += u_derivative * (1/self.freq)
            #Anti-windup
            u_sum = u_proportional + u_integral + u_derivative
            self.windup = False
            if u_sum < self.lmin or u_sum > self.lmax:
                if abs(self.sign(self.err) - self.sign(u_sum)) < 1e-15:
                    self.windup = True
            # Calculates output
            self.u_control =  self.clip(u_sum, self.lmin, self.lmax)
        
    
    def stop(self):
        """Stops thread
        """
        self.stop_event.set()