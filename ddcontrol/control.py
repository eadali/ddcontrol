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
        """Advenced PID controller interface
        
        Args:
            kp (float): Proportional gain of controller
            ki (float): Integral gain of controller
            kd (float): Derivative gain of controller
            kn (float):
            freq(float, optional): 
            lmin, lmax (float, optional):
        """
        Thread.__init__(self)
        #Stop event for thread
        self.sflag = Event()
        # Gains
        self.kp, self.ki, self.kd, self.kn = kp, ki, kd, kn
        #Other parameters
        self.freq, self.lmin, self.lmax = freq, lmin, lmax
        #Integral value of integral term
        #Integral value of derivative term
        self.int, self.fint = 0.0, 0.0
        #Error signal value
        #Control signal value
        self.e, self.u = 0.0, 0.0
        #Previously measured timestamp
        self.ptime = None
        # Anti-Wind Up status
        self.windup = False
        
        
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
    
    
    def __isclose(self, x1, x2, rtol=1e-15):
        return abs(x1-x2) < rtol
        

    def step(self, e, dt):
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


    def run(self):
        #Loop for thread
        self.stop_event.clear()
        while not self.stop_event.is_set():
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
        """Stops thread
        """
        warn('bele vaziyyetin icine soxum.', RuntimeWarning)
        self.stop_event.set()