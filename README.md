# Data-Driven Control
The design of the controller are based entirely on experimental data collected from the plant

## PID controller
PID contoller
A proportional–integral–derivative controller (PID controller or three-term controller)
is a control loop mechanism employing feedback that is widely used inindustrial control systems
and a variety of other applications requiring continuously modulated control.::

```python
class ddc.PIDController(kp, ki, kd, nn, lmin=-inf, lmax=+inf):
```
<img src="https://latex.codecogs.com/gif.latex?u&space;=&space;K_p&space;e(t)&space;&plus;&space;K_i&space;\int_{0}^{t}&space;e(t)&space;&plus;&space;\delta" title="u = K_p e(t) + K_i \int_{0}^{t} e(t) + \delta" />

## Roadmap
- Real time PID controller tuning
- Real time frequency response analysis
- Implementation of state observer