# Data-Driven Control
Control Theory for humans.  
The design of the controller are based entirely on experimental data collected from the plant.

## Simple PID Example
```python
from ddcontrol import PIDController, MSDModel
from time import sleep

#Creates PID controller and test model
pid = PIDController(8.0, 0.01, 8.0, 10.0)
mdl = MSDModel(1.0, 1.0, 0.2, [0.4, 0.0])
pid.start()

#Control loop
u_control = 0.0
print('Press ctrl-c to break loop')
while True:
    try:
        y_meas = mdl.update(u_control)
        u_control = pid.update(-y_meas)
        sleep(0.01)
    except KeyboardInterrupt:
        break
    except Exception as msg:
        print(msg)

pid.stop()
pid.join()
```

## PID controller
```python
class ddcontrol.PIDController(kp, ki, kd, kn, freq=10, lmin=-inf, lmax=+inf):
```
A proportional–integral–derivative controller (PID controller or three-term controller)
is a control loop mechanism employing feedback that is widely used in industrial control systems
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

**Methods**  
**start(self)**  
&nbsp;Starts PID controller thread  
**stop(self)**  
&nbsp;Stops PID controller thread
**update(self, err)**  
&nbsp;Updates PID controller err value  


## Roadmap
- Real time PID controller tuning
- Real time frequency response analysis  