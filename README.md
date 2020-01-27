# Data-Driven Control
Control Theory for humans.  
The PID controller design based entirely on experimental data collected from the plant.

## Simple PID Example
```python
from ddcontrol import PIDController, MSDModel
from time import sleep

#Creates PID controller and test model
pid = PIDController(8.0, 0.01, 8.0, 10.0)
mdl = MSDModel(1.0, 1.0, 0.2, [0.4, 0.0])

#Control loop
pid.start()
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

#Stops PID controller
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
Mathematical equation of PID controller:  
<img src="https://latex.codecogs.com/gif.latex?u(t)&space;=&space;K_p&space;e(t)&space;&plus;&space;K_i&space;\int_{0}^{t}&space;e(t)&space;&plus;&space;\delta(t)" title="u(t) = K_p e(t) + K_i \int_{0}^{t} e(t) + \delta(t)" />  
<img src="https://latex.codecogs.com/gif.latex?\delta(t)&space;=&space;N&space;(K_d&space;e(t)-\int_{0}^{t}&space;\delta(t))" title="\delta(t) = N (K_d e(t)-\int_{0}^{t} \delta(t))" />

**Parameters**  
&nbsp;**kp : float**  
&nbsp;&nbsp;&nbsp;Proportional gain of controller  

&nbsp;**ki : float**  
&nbsp;&nbsp;&nbsp;Integral gain of controller  

&nbsp;**kd : float**  
&nbsp;&nbsp;&nbsp;Derivative gain of controller  

&nbsp;**kn : float**  
&nbsp;&nbsp;&nbsp;Derivative filter gain of controller  

&nbsp;**freq : float (default=10)**  
&nbsp;&nbsp;&nbsp;Frequency for controller  

&nbsp;**lmin : float (default=numpy.inf)**  
&nbsp;&nbsp;&nbsp;Lower limit for controller output  

&nbsp;**lmax : float (default=-numpy.inf)**  
&nbsp;&nbsp;&nbsp;Higher limit for controller output  

**Methods**  
&nbsp;**start(self) -> None**  
&nbsp;&nbsp;&nbsp;Starts PID controller thread  

&nbsp;**stop(self) -> None**  
&nbsp;&nbsp;&nbsp;Stops PID controller thread  

&nbsp;**update(self, err) -> u_control**  
&nbsp;&nbsp;&nbsp;Updates PID controller error signal value and returns control signal value


## Roadmap
- Real time PID controller tuning
- Real time frequency response analysis  