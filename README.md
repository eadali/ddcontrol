# Data-Driven Control
The design of the controller are based entirely on experimental data collected from the plant.

## Simple PID Example
```python
from ddc import PIDController
from test_ddc import IPModel

#Creates PID controller and test model
pid = PIDController(kp=8.0, ki=0.01, kd=0.8, kn=10.0)
mdl = IPModel(0.4, 5.0, [0.5, 0.0])

#Control loop
u_control = 0
print('Press ctrl-c to break loop')
while True:
    y_meas = mdl.update(u_control)
    u_control = pid.update(-y_meas)
```

## PID controller
```python
class ddc.PIDController(kp, ki, kd, kn, freq=10, lmin=-inf, lmax=+inf):
```
A proportional–integral–derivative controller (PID controller or three-term controller)
is a control loop mechanism employing feedback that is widely used inindustrial control systems
and a variety of other applications requiring continuously modulated control[wikipedia.org].  
Mathematical control function:  
<img src="https://latex.codecogs.com/gif.latex?u(t)&space;=&space;K_p&space;e(t)&space;&plus;&space;K_i&space;\int_{0}^{t}&space;e(t)&space;&plus;&space;\delta(t)" title="u(t) = K_p e(t) + K_i \int_{0}^{t} e(t) + \delta(t)" />  
<img src="https://latex.codecogs.com/gif.latex?\delta(t)&space;=&space;N&space;(K_d&space;e(t)-\int_{0}^{t}&space;\delta(t))" title="\delta(t) = N (K_d e(t)-\int_{0}^{t} \delta(t))" />

**Parameters**  
**kp : float**  
&nbsp;Proportional gain of controller  

**ki : float**  
&nbsp;Integral gain of controller  

**kd : float**  
&nbsp;Derivative gain of controller  

**kn : float**  
&nbsp;Derivative filter gain of controller  

**freq : float (default=10)**  
&nbsp;Frequency for controller  

**lmin : float (default=numpy.inf)**  
&nbsp;Lower limit for controller output  

**lmax : float (default=-numpy.inf)**  
&nbsp;Higher limit for controller output  

## Roadmap
- Real time PID controller tuning
- Real time frequency response analysis
- Implementation of state observer

