# -*- coding: utf-8 -*-
"""
Created on Fri Feb 14 17:35:00 2020

@author: ERADALI
"""

from ddcontrol.model import TransferFunction
from ddcontrol.control import pidopt
import numpy as np
import matplotlib.pyplot as plt
import time

#Creates transfer function
tf = TransferFunction([1.0], [1.0,10.0,20.0], udelay=0.1)

#Predicts transfer function
pid, _ = pidopt(tf)
ref = 1.0

#Control loop
history = []
u = 0.0
pid.start()
start = time.time()
for _ in range(1000):
    t = time.time() - start
    y = tf.step(t, u)
    u = pid.update(ref-y)
    history.append([t,y])
    time.sleep(0.001)

#Stops PID controller
pid.stop()
pid.join()

#Plots result
np_hist = np.array(history)
fig, ax = plt.subplots()
ax.plot(np_hist[:,0], np_hist[:,1], '.-')
ax.grid()
plt.show()