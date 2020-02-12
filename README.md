<a href='https://ddcontrol.readthedocs.io/en/latest/?badge=latest'>
    <img src='https://readthedocs.org/projects/ddcontrol/badge/?version=latest' alt='Documentation Status' />
</a>
<a href='https://travis-ci.com/eadali/ddcontrol'>
    <img src='https://travis-ci.com/eadali/ddcontrol.svg?branch=master' alt='Build Status' />
</a>  

# Data-Driven Control
Control Theory for humans.  
The PID controller design based entirely on experimental data collected from the plant.

## Simple PID Example
```python
from ddcontrol.model import TransferFunction
from ddcontrol.control import PIDController
import numpy as np
import matplotlib.pyplot as plt
import time

#Creates PID controller and test model
tf = TransferFunction([1.0], [1.0,10.0,20.0])
pid = PIDController(kp=30, ki=70.0, kd=0.0, kn=0.0)
ref = 1.0

#Control loop
pid.start()
y, u = np.zeros(900), 0.0
start = time.time()
for index in range(y.size):
    t = time.time() - start
    y[index] = tf.step(t, u)
    u = pid.update(ref-y[index])
    time.sleep(0.001)

#Stops PID controller
pid.stop()
pid.join()

#Plots result
fig, ax = plt.subplots()
ax.plot(y)
ax.grid()
plt.show()
```

## Simple PID optimization
```python
from ddcontrol.model import TransferFunction
from ddcontrol.control import pidopt

#Creates transfer function
tf = TransferFunction([1.0], [1.0,10.0,20.0])

#Predicts transfer function
pid, _ = pidopt(tf)
print('Optimized PID gains..:', pid.kp, pid.ki, pid.kd, pid.kn)
```

## Simple Transfer Function Estimation
```python
from ddcontrol.model import TransferFunction, tfest
import numpy as np

#Creates a transfer function and input output data
tf = TransferFunction([1.0], [1.0,10.0,20.0])
t, y, u = np.linspace(0,10,101), np.zeros(101), np.ones(101)
for index in range(t.size):
    y[index] = tf.step(t[index], u[index])

#Predicts transfer function
tf, _ = tfest(t, y, u, np=2, nz=0)
print('Transfer function numerator coeffs..:', tf.num)
print('Transfer function denumerator coeffs..:', tf.den)
```

## Installation
To install using pip:  
```
pip install ddcontrol
```

## Roadmap
- tfest and pidopt functions are very slow. Performance improvement.
- Real time PID controller tuning.
