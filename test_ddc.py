#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 10 19:56:26 2020

@author: eadali
"""

from ddc import Integral, PIDController
from numpy import asarray, zeros, absolute,cumsum
from time import sleep
from scipy.integrate import odeint


class Model:
    def __init__(self, omega, init):
        """Inits mass-spring model
        """
        self.omega = omega
        self.x_0 = init

    def ode(self, x, t, u):
        """Dynamic equations of mass-spring
        """
        pos, vel = x
        dxdt = [vel,  -self.omega**2*pos+u]
        return dxdt

    def update(self, u):
        """Interface function for mass-spring model
        """
        x = odeint(self.ode, self.x_0, [0.0, 0.1], args=(u,))
        self.x_0 = x[1]
        return x[1,0]


def test_Integral():
    """Test of Integral Class
    """
    sequence = asarray([0.0,1.0,1.0,1.0,1.0])
    result = zeros(sequence.shape)
    expected = cumsum(sequence)
    integral = Integral()
    for index in range(sequence.shape[0]):
        result[index] = integral.update(sequence[index])
        sleep(1)

    assert absolute(result-expected).sum() < 0.02


def test_PIDController_P():
    """Test of PIDController P gain
    """
    pid = PIDController(1.0, 0.0, 0.0, 0.0)
    err = asarray([0.0, 2.0, 4.0, 8.0])
    u_cont = zeros(err.shape)
    expected = asarray([0.0, 2.0, 4.0, 8.0])
    for index in range(err.shape[0]):
        u_cont[index] = pid.update(err[index])

    assert absolute(expected-u_cont).sum() < 0.0001


def test_PIDController_I():
    """Test of PIDController I gain
    """
    pid = PIDController(0.0, 1.0, 0.0, 0.0)
    err = asarray([0.0, 2.0, 4.0, 8.0])
    u_cont = zeros(err.shape)
    expected = asarray([0.0, 0.2, 0.6, 1.4])
    for index in range(err.shape[0]):
        u_cont[index] = pid.update(err[index])
        sleep(0.1)

    assert absolute(expected-u_cont).sum() < 0.01


def test_PIDController_D():
    """Test of PIDController D gain
    """
    pid = PIDController(0.0, 0.0, 1000000.0, 0.000001)
    err = asarray([0.0, 2.0, 4.0, 8.0])
    u_cont = zeros(err.shape)
    expected = asarray([0.0, 2.0, 4.0, 8.0])
    for index in range(err.shape[0]):
        u_cont[index] = pid.update(err[index])
        sleep(0.1)

    assert absolute(expected-u_cont).sum() < 0.01


def test_PIDController_N():
    """Test of PIDController N gain
    """
    pid = PIDController(0.0, 0.0, 0.000001, 1.0)
    err = asarray([0.0, 2.0, 4.0, 8.0])
    u_cont = zeros(err.shape)
    expected = asarray([0.0, 0.0, 0.0, 0.0])
    for index in range(err.shape[0]):
        u_cont[index] = pid.update(err[index])
        sleep(0.1)

    assert absolute(expected-u_cont).sum() < 0.01


def test_PIDController():
    """Test of PIDController
    """
    mdl = Model(2.0, [1.0,1.0])
    pid = PIDController(0.0, 0.0, -1.0, 1000.0)
    y = zeros(400)
    u_control = 0.0
    u = zeros(400) ##TODO: delete this line
    for index in range(y.shape[0]):
        y[index] = mdl.update(u_control)
        u_control = pid.update(y[index])
        u[index] = u_control ##TODO:delete this line

    ## TODO: delete these lines
    from numpy import diff
    from matplotlib import pyplot
    pyplot.subplot(2,1,1)
    pyplot.plot(y)
    pyplot.plot(u)
    pyplot.subplot(2,1,2)
#    pyplot.plot(diff(y), 'r')
    pyplot.show()
    #############################

    assert (absolute(y)<0.04).all()
#
#
#def test_PIDTuner():
#    ## TODO: implement this funciton
#    assert False
#
#
#def test_FrequencyResponse():
#    ## TODO: implement this function
#    assert False