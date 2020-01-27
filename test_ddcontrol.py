#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 10 19:56:26 2020

@author: eadali
"""
from ddcontrol import PIDController, MSDModel
from numpy import asarray, zeros, absolute
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