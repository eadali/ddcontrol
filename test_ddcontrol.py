#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 10 19:56:26 2020

@author: eadali
"""
from ddcontrol.integrate import CInterp1d, DDE
from ddcontrol.model import StateSpace, TransferFunction, tfest
from ddcontrol.control import PIDController
from numpy import zeros, absolute, ones, linspace, argmax
from scipy.signal import lsim2, lti
from time import time, sleep


#def test_CInterp1d():
#    """Test of CInterp1d Class
#    """
#    f = CInterp1d(lambda x: [0.0], 0.0)
#    f.append(1.0, [1.0])
#    check1 = absolute(f(0.0)) < 1e-4
#    check2 = absolute(f(0.5)-0.5) < 1e-4
#    check3 = absolute(f(1.5)-1.5) < 1e-4
#    assert check1 and check2 and check3
#
#
#def test_DDE():
#    """Test of DDE Class
#    """
#    f = lambda t, x: [-x(t-1.0)]
#    solver = DDE(f)
#    solver.set_integrator('dopri5')
#    g = lambda t: [1.0]
#    t = linspace(0,10,101)
#    y = zeros(t.shape)
#    solver.set_initial_value(g, 0.0)
#    for index in range(t.shape[0]):
#        y[index] = solver.integrate(t[index])
#    check1 = absolute(y[0]-1.0) < 0.01
#    check2 = (absolute(y[-10:]) < 0.1).all()
#    assert check1 and check2
#
#
#def test_StateSpace():
#    """Test of StateSpace Class
#    """
#    t = linspace(0,10,101)
#    u = ones(t.shape)
#    A, B = [[0.0, 1.0], [0.0, 0.0]], [[0.0], [1.0]]
#    C, D = [[1.0, 0.0]], [[0.0]]
#    scipy_ss = lti(A, B, C, D)
#    _, y_scipy, _ = lsim2(scipy_ss, u, t)
#    ss = StateSpace(A, B, C, D)
#    y_ss = zeros(t.shape)
#    for index in range(t.shape[0]):
#        y_ss[index] = ss.integrate(t[index], u[index])
#    assert (absolute(y_ss-y_scipy) < 0.1).all()
#
#
#def test_StateSpace_udelay():
#    """Test of StateSpace Class with input delay
#    """
#    t = linspace(0,10,101)
#    u = ones(t.shape)
#    A, B = [[0.0, 1.0], [0.0, 0.0]], [[0.0], [1.0]]
#    C, D = [[1.0, 0.0]], [[0.0]]
#    scipy_ss = lti(A, B, C, D)
#    _, y_scipy, _ = lsim2(scipy_ss, u, t)
#    A = [[[0.0, 1.0], [0.0, 0.0]], [[0.0, 0.0], [0.0, 0.0]]]
#    B = [[[0.0], [0.0]], [[0.0], [1.0]]]
#    C = [[[1.0, 0.0]], [[0.0, 0.0]]]
#    D = [[[0.0]]]
#    ss = StateSpace(A, B, C, D, delays=[0.0,1.0])
#    y_ss = zeros(t.shape)
#    for index in range(t.shape[0]):
#        y_ss[index] = ss.integrate(t[index], u[index])
#    assert (absolute(y_ss[argmax(t>=1.0):]-y_scipy[:-argmax(t>=1.0)] < 0.1).all())
#
#
#def test_StateSpace_ydelay():
#    """Test of StateSpace Class with output delay
#    """
#    t = linspace(0,10,101)
#    u = ones(t.shape)
#    A, B = [[0.0, 1.0], [0.0, 0.0]], [[0.0], [1.0]]
#    C, D = [[1.0, 0.0]], [[0.0]]
#    scipy_ss = lti(A, B, C, D)
#    _, y_scipy, _ = lsim2(scipy_ss, u, t)
#    A = [[[0.0, 1.0], [0.0, 0.0]], [[0.0, 0.0], [0.0, 0.0]]]
#    B = [[[0.0], [1.0]], [[0.0], [0.0]]]
#    C = [[[0., 0.]], [[1.0, 0.0]]]
#    D = [[[0.0]]]
#    ss = StateSpace(A, B, C, D, delays=[0.0,1.0])
#    y_ss = zeros(t.shape)
#    for index in range(t.shape[0]):
#        y_ss[index] = ss.integrate(t[index], u[index])
#    assert (absolute(y_ss[argmax(t>=1.0):]-y_scipy[:-argmax(t>=1.0)] < 0.1).all())
#
#
#def test_StateSpace_sdelay():
#    """Test of StateSpace Class with state delay
#    """
#    t = linspace(0,10,101)
#    f = lambda t, x: [-x(t-1.0)]
#    solver = DDE(f)
#    solver.set_integrator('dopri5')
#    g = lambda t: [1.0]
#    y_dde = zeros(t.shape)
#    solver.set_initial_value(g, 0.0)
#    for index in range(t.shape[0]):
#        y_dde[index] = solver.integrate(t[index])
#    A = [[[0.0]], [[-1.0]]]
#    B = [[[0.0]], [[0.0]]]
#    C = [[[1.0]],[[0.0]]]
#    D = [[[0.0]],[[0.0]]]
#    ss = StateSpace(A, B, C, D, delays=[0.0,1.0])
#    ss.set_initial_value(x0=[1.0])
#    y_ss = zeros(t.shape)
#    for index in range(t.shape[0]):
#        y_ss[index] = ss.integrate(t[index], [0.0])
#    assert (absolute(y_ss-y_dde) < 0.1).all()
#
#
#def test_TransferFunction():
#    """Test of TransferFunction Class
#    """
#    t = linspace(0,10,101)
#    u = ones(t.shape)
#    num = [1.0, 3.0, 3.0]
#    den = [1.0, 2.0, 1.0]
#    tf = TransferFunction(num, den)
#    scipy_tf = lti(num, den)
#    _, y_scipy, _ = lsim2(scipy_tf, u, t)
#    y_tf = zeros(t.shape)
#    for index in range(t.shape[0]):
#        y_tf[index] = tf.integrate(t[index], u[index])
#    assert (absolute(y_tf-y_scipy)<0.1).all()
#
#
#def test_TransferFunction_udelay():
#    """Test of TransferFunction Class with input delay
#    """
#    t = linspace(0,10,101)
#    u = ones(t.shape)
#    num = [1.0, 3.0, 3.0]
#    den = [1.0, 2.0, 1.0]
#    tf = TransferFunction(num, den)
#    scipy_tf = lti(num, den)
#    _, y_scipy, _ = lsim2(scipy_tf, u, t)
#    tf = TransferFunction(num, den, 1.0)
#    tf.set_initial_value(0.0)
#    y_tf = zeros(t.shape)
#    for index in range(t.shape[0]):
#        y_tf[index] = tf.integrate(t[index], u[index])
#    assert (absolute(y_tf[argmax(t>=1.0):]-y_scipy[:-argmax(t>=1.0)] < 0.1).all())
#
#
#def test_tfest():
#    """Test of tfest Method
#    """
#    t = linspace(0,10,101)
#    u = ones(t.shape)
#    num = [1.0, 3.0, 3.0]
#    den = [1.0, 2.0, 1.0]
#    scipy_tf = lti(num, den)
#    _, y_scipy, _ = lsim2(scipy_tf, u, t)
#    tf, _ = tfest(t, y_scipy, u, 3, 3)
#    y_tf = zeros(t.shape)
#    for index in range(t.shape[0]):
#        y_tf[index] = tf.integrate(t[index], u[index])
#    assert (absolute(y_tf-y_scipy)<0.1).all()
#    
#    
#def test_tfest_udelay():
#    """Test of tfest Method with udelay
#    """
#    t = linspace(0,10,101)
#    u = ones(t.shape)
#    num = [1.0, 3.0, 3.0]
#    den = [1.0, 2.0, 1.0]
#    scipy_tf = lti(num, den)
#    _, y_scipy, _ = lsim2(scipy_tf, u, t) 
#    y_scipy[argmax(t>=1.0):] = y_scipy[:-argmax(t>=1.0)]
#    y_scipy[:argmax(t>=1.0)] = 1.0
#    tf, _ = tfest(t, y_scipy, u, 3, 3, True)
#    y_tf = zeros(t.shape)
#    for index in range(t.shape[0]):
#        y_tf[index] = tf.integrate(t[index], u[index])
#    from matplotlib import pyplot
#    pyplot.plot(y_scipy)
#    pyplot.plot(y_tf)
#    pyplot.show()
#    assert False
#    assert (absolute(y_tf-y_scipy)<0.1).all()
    

def test_PIDController_P():
    """Test of PIDController Class P gain
    """
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, kn=0.0, freq=10.0)
    pid.start()
    pid.integrate(1.0)
    sleep(0.4)
    u = pid.integrate(1.0)
    pid.stop()
    pid.join()
    assert abs(1.0-u) < 0.001
#
#
#def test_PIDController_I():
#    """Test of PIDController I gain
#    """
#    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, kn=0.0, freq=10.0)
#    pid.start()
#    pid.update(1.0)
#    sleep(1.0)
#    u_cont = pid.update(1.0)
#    pid.stop()
#    pid.join()
#    assert abs(1.0-u_cont) < 0.2
#
#
#def test_PIDController_D():
#    """Test of PIDController D gain
#    """
#    pid = PIDController(kp=0.0, ki=0.0, kd=1.0, kn=1.0, freq=10.0)
#    pid.start()
#    pid.update(1.0)
#    sleep(1.0)
#    u_cont = pid.update(1.0)
#    pid.stop()
#    pid.join()
#    assert abs(0.4-u_cont) < 0.2
#
#
#def test_PIDController_freq():
#    """Test of PIDController frequency
#    """
#    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, kn=0.0, freq=10.0)
#    pid.start()
#    start = time()
#    sleep(1.0)
#    end = pid.past
#    pid.stop()
#    pid.join()
#    assert abs(end-start-1.0) < 0.2
#
#
#def test_PIDController_clamp():
#    """Test of PIDController clamp
#    """
#    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, kn=0.0, freq=10.0, lmin=-0.4, lmax=0.4)
#    pid.start()
#    pid.update(1.0)
#    sleep(1.0)
#    u_cont = pid.update(1.0)
#    check1 = u_cont < 0.5
#    pid.update(-1.0)
#    sleep(1.0)
#    u_cont = pid.update(-1.0)
#    check2 = u_cont > -0.5
#    pid.stop()
#    pid.join()
#    assert check1 and check2
#
#
#def test_PIDController():
#    """Test of PIDController
#    """
#    pid = PIDController(8.0, 0.01, 8.0, 10.0)
#    mdl = MSDModel(1.0, 1.0, 0.2, [0.4, 0.0])
#    y = zeros(400)
#    u_control = 0.0
#    pid.start()
#    for index in range(y.shape[0]):
#        y[index] = mdl.update(u_control)
#        u_control = pid.update(-y[index])
#        sleep(0.01)
#    pid.stop()
#    pid.join()
#    assert (absolute(y[-10:]) < 0.01).all()