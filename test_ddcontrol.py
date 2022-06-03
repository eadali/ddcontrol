#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 10 19:56:26 2020

@author: eadali
"""
from ddcontrol.integrate import CInterp1d, ode, dde
from ddcontrol.model import StateSpace, TransferFunction, tfest
from ddcontrol.control import PIDController, pidopt
from numpy import zeros, absolute, ones, linspace, argmax, sin, pi
from scipy.signal import lsim2, lti
from scipy.integrate import odeint
from time import time, sleep


def test_CInterp1d():
    """Test of CInterp1d Class
    """
    f = CInterp1d(0.0, lambda x: [0.0])
    f.append(1.0, [1.0])
    check1 = absolute(f(0.0)) < 1e-4
    check2 = absolute(f(0.5)-0.5) < 1e-4
    check3 = absolute(f(1.5)-1.5) < 1e-4
    assert check1 and check2 and check3


def test_ODE():
    """Test of ODE Class
    """
    pendulum = lambda t, y: [y[1], -0.25*y[1] - 5.0*sin(y[0])]
    t = linspace(0, 10, 101)
    y0 = [pi - 0.1, 0.0]    
    scipy_solution = odeint(pendulum, y0, t, tfirst=True)
    
    solver = ode(pendulum).set_initial_value(t[0], y0).set_f_params()
    ode_solution = zeros((t.shape[0], len(y0)))
    for i in range(t.shape[0]):
        ode_solution[i] = solver.integrate(t[i])
    check = (absolute(scipy_solution-ode_solution) < 0.1).all()
    assert check


def test_DDE():
    """Test of DDE Class
    """
    f = lambda t, x: [-x(t-1.0)]
    solver = dde(f)
    g = lambda t: [1.0]
    t = linspace(0,10,101)
    y = zeros(t.shape)
    solver.set_initial_value(0.0, g)
    for index in range(t.shape[0]):
        y[index] = solver.integrate(t[index])
    check1 = absolute(y[0]-1.0) < 0.01
    check2 = (absolute(y[-10:]) < 0.1).all()
    assert check1 and check2


def test_StateSpace():
    """Test of StateSpace Class
    """
    t = linspace(0,10,101)
    u = ones(t.shape)
    A, B = [[0.0, 1.0], [0.0, 0.0]], [[0.0], [1.0]]
    C, D = [[1.0, 0.0]], [[0.0]]
    scipy_ss = lti(A, B, C, D)
    _, y_scipy, _ = lsim2(scipy_ss, u, t)
    ss = StateSpace(A, B, C, D)
    y_ss = zeros(t.shape)
    for index in range(t.shape[0]):
        y_ss[index] = ss.step(t[index], u[index])
    assert (absolute(y_ss-y_scipy) < 0.1).all()


def test_StateSpace_udelay():
    """Test of StateSpace Class with input delay
    """
    t = linspace(0,10,101)
    u = ones(t.shape)
    A, B = [[0.0, 1.0], [0.0, 0.0]], [[0.0], [1.0]]
    C, D = [[1.0, 0.0]], [[0.0]]
    scipy_ss = lti(A, B, C, D)
    _, y_scipy, _ = lsim2(scipy_ss, u, t)
    A = [[[0.0, 1.0], [0.0, 0.0]], [[0.0, 0.0], [0.0, 0.0]]]
    B = [[[0.0], [0.0]], [[0.0], [1.0]]]
    C = [[[1.0, 0.0]], [[0.0, 0.0]]]
    D = [[[0.0]]]
    ss = StateSpace(A, B, C, D, delays=[0.0,1.0])
    y_ss = zeros(t.shape)
    for index in range(t.shape[0]):
        y_ss[index] = ss.step(t[index], u[index])
    assert (absolute(y_ss[argmax(t>=1.0):]-y_scipy[:-argmax(t>=1.0)] < 0.1).all())


def test_StateSpace_ydelay():
    """Test of StateSpace Class with output delay
    """
    t = linspace(0,10,101)
    u = ones(t.shape)
    A, B = [[0.0, 1.0], [0.0, 0.0]], [[0.0], [1.0]]
    C, D = [[1.0, 0.0]], [[0.0]]
    scipy_ss = lti(A, B, C, D)
    _, y_scipy, _ = lsim2(scipy_ss, u, t)
    A = [[[0.0, 1.0], [0.0, 0.0]], [[0.0, 0.0], [0.0, 0.0]]]
    B = [[[0.0], [1.0]], [[0.0], [0.0]]]
    C = [[[0., 0.]], [[1.0, 0.0]]]
    D = [[[0.0]]]
    ss = StateSpace(A, B, C, D, delays=[0.0,1.0])
    y_ss = zeros(t.shape)
    for index in range(t.shape[0]):
        y_ss[index] = ss.step(t[index], u[index])
    assert (absolute(y_ss[argmax(t>=1.0):]-y_scipy[:-argmax(t>=1.0)] < 0.1).all())


def test_StateSpace_sdelay():
    """Test of StateSpace Class with state delay
    """
    t = linspace(0,10,101)
    f = lambda t, x: [-x(t-1.0)]
    solver = dde(f)
    g = lambda t: [1.0]
    y_dde = zeros(t.shape)
    solver.set_initial_value(0.0, g)
    for index in range(t.shape[0]):
        y_dde[index] = solver.integrate(t[index])
    A = [[[0.0]], [[-1.0]]]
    B = [[[0.0]], [[0.0]]]
    C = [[[1.0]],[[0.0]]]
    D = [[[0.0]],[[0.0]]]
    ss = StateSpace(A, B, C, D, delays=[0.0,1.0])
    ss.set_initial_value(x0=[1.0])
    y_ss = zeros(t.shape)
    for index in range(t.shape[0]):
        y_ss[index] = ss.step(t[index], [0.0])
    assert (absolute(y_ss-y_dde) < 0.1).all()


def test_TransferFunction():
    """Test of TransferFunction Class
    """
    t = linspace(0,10,101)
    u = ones(t.shape)
    num = [1.0, 3.0, 3.0]
    den = [1.0, 2.0, 1.0]
    tf = TransferFunction(num, den)
    scipy_tf = lti(num, den)
    _, y_scipy, _ = lsim2(scipy_tf, u, t)
    y_tf = zeros(t.shape)
    for index in range(t.shape[0]):
        y_tf[index] = tf.step(t[index], u[index])
    assert (absolute(y_tf-y_scipy)<0.1).all()


def test_TransferFunction_udelay():
    """Test of TransferFunction Class with input delay
    """
    t = linspace(0,10,101)
    u = ones(t.shape)
    num = [1.0, 3.0, 3.0]
    den = [1.0, 2.0, 1.0]
    tf = TransferFunction(num, den)
    scipy_tf = lti(num, den)
    _, y_scipy, _ = lsim2(scipy_tf, u, t)
    tf = TransferFunction(num, den, 1.0)
    tf.set_initial_value(0.0)
    y_tf = zeros(t.shape)
    for index in range(t.shape[0]):
        y_tf[index] = tf.step(t[index], u[index])
    assert (absolute(y_tf[argmax(t>=1.0):]-y_scipy[:-argmax(t>=1.0)] < 0.1).all())


def test_tfest():
    """Test of tfest Method
    """
    t = linspace(0,10,101)
    u = ones(t.shape)
    num = [1.0, 3.0, 3.0]
    den = [1.0, 2.0, 1.0]
    scipy_tf = lti(num, den)
    _, y_scipy, _ = lsim2(scipy_tf, u, t)
    tf, _ = tfest(t, y_scipy, u, np=3, nz=3)
    y_tf = zeros(t.shape)
    for index in range(t.shape[0]):
        y_tf[index] = tf.step(t[index], u[index])
    assert (absolute(y_tf-y_scipy)<0.1).all()
    
    
def test_tfest_udelay():
    """Test of tfest Method with udelay
    """
    t = linspace(0,10,101)
    u = ones(t.shape)
    num = [1.0, 3.0, 3.0]
    den = [1.0, 2.0, 1.0]
    tf = TransferFunction(num, den, udelay=1.0)
    y = zeros(t.shape)
    for index in range(t.shape[0]):
        y[index] = tf.step(t[index], u[index])
    _tf, _ = tfest(t, y, u, np=3, nz=3)
    _y = zeros(t.shape)
    for index in range(t.shape[0]):
        _y[index] = tf.step(t[index], u[index])
    assert (absolute(_y-y)<0.1).all()


def test_PIDController_P():
    """Test of PIDController Class P gain
    """
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, kn=0.0)
    u = pid.step(1.0, 1.0)
    assert abs(u-1.0) < 1e-4
    
    
def test_PIDController_I():
    """Test of PIDController Class I gain
    """
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, kn=0.0)
    u = pid.step(1.0, 1.0)
    assert abs(u-1.0) < 1e-4

    
def test_PIDController_D():
    """Test of PIDController Class D gain
    """
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0, kn=1.0)
    u = pid.step(1.0, 1.0)
    assert abs(u-1.0) < 1e-4


def test_PIDController_clamp():
    """Test of PIDController Class clamp
    """
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, kn=0.0, lmin=-1.0, lmax=1.0)
    check1 = pid.step(10.0, 1.0) < 1.1
    check2 = pid.step(-100.0, 1.0) > -1.1
    assert check1 and check2


def test_PIDController():
    """Test of PIDController Class
    """
    tf = TransferFunction([1.0], [1.0,10.0,20.0])
    pid = PIDController(kp=30, ki=70.0, kd=1.0, kn=1.0)
    pid.start()
    y, u = zeros(900, 'float32'), 0.0
    start = time()
    for index in range(y.size):
        t = time() - start
        y[index] = tf.step(t, u)
        u = pid.update(1-y[index])
        sleep(0.001)
    pid.stop()
    pid.join()
    assert (absolute(y[-10:] - 1.0) < 0.01).all()
    
    
def test_pidopt():
    """Test of tunePID Method
    """
    t = linspace(0,10,101)
    tf = TransferFunction([1.0], [1.0,10.0,20.0])
    pid, _ = pidopt(tf)
    y, u = zeros(101, 'float32'), 0.0
    for index in range(1,  t.size):
        y[index] = tf.step(t[index], u)
        dt = t[index] - t[index-1]
        u = pid.step(dt, 1.0-y[index])
    assert (absolute(y[-10:] - 1.0) < 0.04).all()