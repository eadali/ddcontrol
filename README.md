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
![equation](http://www.sciweavers.org/tex2img.php?eq=1%2Bsin%28mc%5E2%29&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=)
## Roadmap
- Real time PID controller tuning
- Real time frequency response analysis
- Implementation of state observer