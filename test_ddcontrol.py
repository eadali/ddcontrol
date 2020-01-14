#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 10 19:56:26 2020

@author: eadali
"""

from ddcontrol import PIDController
from numpy import asarray, zeros, absolute, sin
from time import time
from scipy.integrate import odeint


class IPModel:
    def __init__(self, b, c, x_0):
        """Inits mass-spring model
        """
        self.b = b
        self.c = c
        self.x_0 = x_0
        self.past = None

    def ode(self, x, t, u):
        """Dynamic equations of mass-spring
        """
        # ODE of pendulum
        theta, omega = x
        dxdt = [omega, -self.b*omega + self.c*sin(theta) + u]
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


def test_PIDController_P():
    """Test of PIDController P gain
    """
    pid = PIDController(1.0, 0.0, 0.0, 0.0)
    err = asarray([0.0, 2.0, 4.0, 8.0])
    u_cont = zeros(err.shape)
    expected = asarray([0.0, 2.0, 4.0, 8.0])
    for index in range(err.shape[0]):
        u_cont[index] = pid.update(err[index])
    assert absolute(expected-u_cont).sum() < 0.001


def test_PIDController_I():
    """Test of PIDController I gain
    """
    pid = PIDController(0.0, 1.0, 0.0, 0.0)
    err = asarray([0.0, 2.0, 4.0, 8.0])
    u_cont = zeros(err.shape)
    expected = asarray([0.0, 0.2, 0.6, 1.4])
    for index in range(err.shape[0]):
        u_cont[index] = pid.update(err[index])
    assert absolute(expected-u_cont).sum() < 0.001


def test_PIDController_D():
    """Test of PIDController D gain
    """
    pid = PIDController(0.0, 0.0, 1.0, 1.0)
    err = asarray([0.0, 2.0, 4.0, 8.0])
    u_cont = zeros(err.shape)
    expected = asarray([0.0, 2.0, 3.8, 7.42])
    for index in range(err.shape[0]):
        u_cont[index] = pid.update(err[index])
    assert absolute(expected-u_cont).sum() < 0.001


def test_PIDController_freq():
    """Test of PIDController frequency
    """
    freq = 10
    pid = PIDController(0.0, 0.0, 0.0, 0.0, freq)
    pid.update(0.0)
    start = time()
    pid.update(0.0)
    assert absolute((time() - start) - (1 / freq)) < 0.01


def test_PIDController_clamp():
    """Test of PIDController clamp
    """
    pid = PIDController(0.0, 1.0, 0.0, 0.0, lmin=-1.0, lmax=1.0)
    err = zeros(64)
    err[:32] = -1.0
    err[32:] = 1.0
    u_cont = zeros(err.shape)
    for index in range(err.shape[0]):
        u_cont[index] = pid.update(err[index])
    assert (absolute(u_cont) < 1.0+1e-15).all()


def test_PIDController():
    """Test of PIDController
    """
    pid = PIDController(8.0, 0.01, 0.8, 10.0)
    mdl = IPModel(0.4, 5.0, [0.5, 0.0])
    y = zeros(100)
    u_control = 0.0
    for index in range(y.shape[0]):
        y[index] = mdl.update(u_control)
        u_control = pid.update(-y[index])
    assert (absolute(y[-10:]) < 0.04).all()