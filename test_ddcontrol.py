#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 10 19:56:26 2020

@author: eadali
"""
from ddcontrol import PIDController, MSDModel, Interpolate, DDE, StateSpace
from numpy import zeros, absolute, ones, linspace
from scipy.signal import lsim2, lti
from time import time, sleep


def test_PIDController_P():
    """Test of PIDController P gain
    """
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, kn=0.0, freq=10.0)
    pid.start()
    pid.update(1.0)
    sleep(1.0)
    u_cont = pid.update(1.0)
    pid.stop()
    pid.join()
    assert abs(1.0-u_cont) < 0.001


def test_PIDController_I():
    """Test of PIDController I gain
    """
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, kn=0.0, freq=10.0)
    pid.start()
    pid.update(1.0)
    sleep(1.0)
    u_cont = pid.update(1.0)
    pid.stop()
    pid.join()
    assert abs(1.0-u_cont) < 0.2


def test_PIDController_D():
    """Test of PIDController D gain
    """
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0, kn=1.0, freq=10.0)
    pid.start()
    pid.update(1.0)
    sleep(1.0)
    u_cont = pid.update(1.0)
    pid.stop()
    pid.join()
    assert abs(0.4-u_cont) < 0.2


def test_PIDController_freq():
    """Test of PIDController frequency
    """
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, kn=0.0, freq=10.0)
    pid.start()
    start = time()
    sleep(1.0)
    end = pid.past
    pid.stop()
    pid.join()
    assert abs(end-start-1.0) < 0.2


def test_PIDController_clamp():
    """Test of PIDController clamp
    """
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, kn=0.0, freq=10.0, lmin=-0.4, lmax=0.4)
    pid.start()
    pid.update(1.0)
    sleep(1.0)
    u_cont = pid.update(1.0)
    check1 = u_cont < 0.5
    pid.update(-1.0)
    sleep(1.0)
    u_cont = pid.update(-1.0)
    check2 = u_cont > -0.5
    pid.stop()
    pid.join()
    assert check1 and check2


def test_PIDController():
    """Test of PIDController
    """
    pid = PIDController(8.0, 0.01, 8.0, 10.0)
    mdl = MSDModel(1.0, 1.0, 0.2, [0.4, 0.0])
    y = zeros(400)
    u_control = 0.0
    pid.start()
    for index in range(y.shape[0]):
        y[index] = mdl.update(u_control)
        u_control = pid.update(-y[index])
        sleep(0.01)
    pid.stop()
    pid.join()
    assert (absolute(y[-10:]) < 0.01).all()


def test_Interpolate():
    """Test of Interpolate
    """
    def g(x):
        return 0.0
    interp = Interpolate(g, 0.0)
    interp.append(1.0, 1.0)
    check1 = absolute(interp(0.0)) < 0.01
    check2 = absolute(interp(0.5)-0.5) < 0.01
    check3 = absolute(interp(1.5)-1.5) < 0.01
    assert check1 and check2 and check3


def test_DDE():
    """Test of DDE
    """
    def f(t, x, u):
        return -x(t-1.0)
    def g(t):
        return 1.0
    dde = DDE(f)
    dde.set_integrator('dopri5')
    dde.set_initial_value(g, 0.0)
    timestamps = linspace(0,20,200)
    y = zeros(timestamps.shape)
    for index in range(timestamps.shape[0]):
        dde.set_f_params((0.0,))
        y[index] = dde.integrate(timestamps[index])
    check1 = absolute(y[0]-1.0) < 0.01
    check2 = (absolute(y[-10:]) < 0.01).all()
    assert check1 and check2


def test_StateSpace():
    """Test of StateSpace
    """
    A = [[0., 1.], [0., 0.]]
    B = [[0.], [1.]]
    C = [[1., 0.]]
    D = 0.
    ss = StateSpace(A, B, C, D)
    scipy_ss = lti(A, B, C, D)
    timestamps = linspace(0,10,50)
    u_cont = ones(timestamps.shape)
    y_meas = zeros(timestamps.shape)
    for index in range(timestamps.shape[0]):
        y_meas[index] = ss.update(timestamps[index], u_cont[index])
    _, y, _ = lsim2(scipy_ss, u_cont, timestamps)
    assert (absolute(y_meas-y)<0.1).all()


def test_StateSpace_idelay():
    """Test of StateSpace with input delay
    """
    A, B, C, D = [[0., 1.], [0., 0.]], [[0.], [1.]], [[1., 0.]], 0.
    scipy_ss = lti(A, B, C, D)
    A = [[[0., 1.], [0., 0.]], [[0., 0.], [0., 0.]]]
    B = [[[0.], [0.]], [[0.], [1.]]]
    C = [[[1., 0.]], [[0., 0.]]]
    D = 0.
    ss = StateSpace(A, B, C, D, delays=[0.0,1.0])
    timestamps = linspace(0,10,50)
    u_cont = ones(timestamps.shape)
    y_meas = zeros(timestamps.shape)
    for index in range(timestamps.shape[0]):
        y_meas[index] = ss.update(timestamps[index], u_cont[index])
    _, y, _ = lsim2(scipy_ss, u_cont, timestamps)
    assert (absolute(y_meas[5:]-y[:-5]< 0.4).all())


def test_StateSpace_odelay():
    """Test of StateSpace with output delay
    """
    A, B, C, D = [[0., 1.], [0., 0.]], [[0.], [1.]], [[1., 0.]], 0.
    scipy_ss = lti(A, B, C, D)
    A = [[[0., 1.], [0., 0.]], [[0., 0.], [0., 0.]]]
    B = [[[0.], [1.]], [[0.], [0.]]]
    C = [[[0., 0.]], [[1., 0.]]]
    D = 0.
    ss = StateSpace(A, B, C, D, delays=[0.0,1.0])
    timestamps = linspace(0,10,50)
    u_cont = ones(timestamps.shape)
    y_meas = zeros(timestamps.shape)
    for index in range(timestamps.shape[0]):
        y_meas[index] = ss.update(timestamps[index], u_cont[index])
    _, y, _ = lsim2(scipy_ss, u_cont, timestamps)
    assert (absolute(y_meas[5:]-y[:-5]< 0.4).all())


def test_StateSpace_sdelay():
    """Test of StateSpace with state delay
    """
    def f(t, x):
        return -x(t-1.0)
    def g(t):
        return 1.0
    dde = DDE(f)
    dde.set_integrator('dopri5')
    dde.set_initial_value(g, 0.0)
    timestamps = linspace(0,10,50)
    y = zeros(timestamps.shape)
    for index in range(timestamps.shape[0]):
        y[index] = dde.integrate(timestamps[index])

    A = [[[0.]], [[-1.]]]
    B = [[[0.]], [[0.]]]
    C = [[[1.]],[[0.]]]
    D = [[[0.]],[[0.]]]
    ss = StateSpace(A, B, C, D, x0=[1.0], delays=[0.0,1.0])
    y_meas = zeros(timestamps.shape)
    for index in range(timestamps.shape[0]):
        y_meas[index] = ss.update(timestamps[index], [0.0])
    assert (absolute(y_meas-y)<0.1).all()