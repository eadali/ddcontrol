# -*- coding: utf-8 -*-
"""
Created on Wed Jan 15 10:06:42 2020

@author: ERADALI
"""
from scipy.integrate import ode
from numpy import array, zeros, ndarray, expand_dims
from scipy.interpolate import interp1d
from collections import deque
from time import time
from scipy.integrate import odeint



class Interpolate:
    def __init__(self, g, x0=0, lsize=1e4):
        """Inits Interpolate
        #Argumenst
            g: Initial function
            x0: Initial function initial input value
        """
        self.g = g
        self.x0 = x0
        self.lsize = lsize
        self.x = deque([self.x0-2e-9, self.x0-1e-9])
        self.y = deque([array(self.g(self.x0-2e-9)), array(self.g(self.x0-1e-9))])
        self.interp1d = interp1d(self.x, self.y, axis=0, fill_value='extrapolate')


    def __call__(self, x):
        """Calculates interpolated values
        #Argumenst
            x: Input value
        #Returns
            Interpolated value
        """
        if x < self.x0:
            return self.g(x)
        else:
            return self.interp1d(x)


    def append(self, x_new, y_new):
        """Appends new values to interpolation
        """
        self.x.append(x_new)
        self.y.append(y_new)
        if len(self.x) > self.lsize:
            self.x.popleft()
            self.y.popleft()
        self.interp1d = interp1d(self.x, self.y, axis=0, fill_value='extrapolate')



class DDE(ode):
    """Thanks to http://zulko.github.io/
    """
    def __init__(self, f, jac=None):
        def w(t, y, args):
            return f(t, self.interp, *args)
        ode.__init__(self, w, jac)
        self.set_f_params(())

    def set_initial_value(self, g, t0=0.0):
        """Set initial conditions y(t) = y."""
        self.t0 = t0
        self.interp = Interpolate(g, t0)
        ode.set_initial_value(self, g(t0), t0)


    def integrate(self, t, step=False, relax=False):
        """Find y=y(t), set y as an initial condition, and return y.
        Parameters
        ----------
        t : float
            The endpoint of the integration step.
        step : bool
            If True, and if the integrator supports the step method,
            then perform a single integration step and return.
            This parameter is provided in order to expose internals of
            the implementation, and should not be changed from its default
            value in most cases.
        relax : bool
            If True and if the integrator supports the run_relax method,
            then integrate until t_1 >= t and return. ``relax`` is not
            referenced if ``step=True``.
            This parameter is provided in order to expose internals of
            the implementation, and should not be changed from its default
            value in most cases.
        Returns
        -------
        y : float
            The integrated value at t
        """
        if t < self.t0:
            y = self.interp(t)
        else:
            y = ode.integrate(self, t, step, relax)
            self.interp.append(t, y)
        return y



class StateSpace:
    def __init__(self, A, B, C, D, x0=None, u0=None, delays=None):
        """Inits state space model
        #Arguments:
            A, B, C, D: State space matrices
            x0: Initial condition for states
            delays: Delay values
        """
        #Sets state space matrices
        self.A = array(A, 'float32')
        if self.A.ndim == 2:
            self.A = expand_dims(self.A, 0)
        self.B = array(B, 'float32')
        if self.B.ndim == 2:
            self.B = expand_dims(self.B, 0)
        self.C = array(C, 'float32')
        if self.C.ndim == 2:
            self.C = expand_dims(self.C, 0)
        self.D = array(D, 'float32')
        if self.D.ndim == 2:
            self.D = expand_dims(self.D, 0)
        #Sets states initial
        if x0 is None:
            x0 = lambda t: zeros(self.A.shape[2], 'float32')
        elif isinstance(x0, (list, tuple, ndarray)):
            xc = x0.copy()
            x0 = lambda t: array(xc, 'float32' )
        #Sets input initial
        if u0 is None:
            u0 = lambda t: zeros(self.B.shape[2], 'float32')
        elif isinstance(u0, (list, tuple, ndarray)):
            uc = u0.copy()
            u0 = lambda t: array(uc, 'float32')
        #Sets delays
        if delays is None:
            self.delays = array([0.0], 'float32')
        elif isinstance(delays, list) or isinstance(delays, ndarray):
            self.delays = array(delays, 'float32')
        #Creates input function
        self.u = Interpolate(u0, 0.0)
        #Creates states function
        self.x = Interpolate(x0, 0.0)
        #Sets DDE
        self.solver = DDE(self.equation)
        self.solver.set_integrator('dopri5')
        self.solver.set_initial_value(x0, 0.0)


    def equation(self, t, x, u):
        """Delay Differential Equation
        #Arguments:
            t: Timestamp
            x: States of delay differential equation
            u: Input of delay differential equation
        #Returns
            Derivative of states
        """
        ##TODO: find a clean solution
        #Creates state matrix
        xd = zeros((self.A.shape[0],1,self.A.shape[2]), 'float32')
        for index, dly in enumerate(self.delays):
            xd[index,:,:] = x(t-dly)
        #Creates input matrix
        ud = zeros((self.B.shape[0],1,self.B.shape[2]), 'float32')
        for index, dly in enumerate(self.delays):
            ud[index,:,:] = u(t-dly)
        ##
        dxdt = (self.A*xd).sum(axis=(0,2)) + (self.B*ud).sum(axis=(0,2))
        return dxdt


    def update(self, t, u_new):
        """Updates state space model
        #Arguments:
            t: Timestamp
            u: Input value
        #Returns
            Output of state space model
        """
        #Appends new input value
        self.u.append(t, u_new)
        #Sets input
        self.solver.set_f_params((self.u,))
        #Solves dde
        x_new = self.solver.integrate(t)
        self.x.append(t, x_new)
        ##TODO: find a clean solution
        #Creates state matrix
        xd = zeros((self.A.shape[0],1,self.A.shape[2]), 'float32')
        for index, dly in enumerate(self.delays):
            xd[index,:,:] = self.x(t-dly)
        #Creates input matrix
        ud = zeros((self.B.shape[0],1,self.B.shape[2]), 'float32')
        for index, dly in enumerate(self.delays):
            ud[index,:,:] = self.u(t-dly)
        y = (self.C*xd).sum(axis=(0,2)) + (self.D*ud).sum(axis=(0,2))
        return y



##TODO: Reframe model
class MSDModel:
    def __init__(self, m, k, b, x0):
        """Inits mass-spring-damper model
        #Arguments:
            m: Mass value
            k: Spring value
            b: Damper value
            x0: Initial states
        """
        #System Parameters
        self.m, self.k, self.b = m, k, b
        #Initial states
        self.x0 = x0
        #Previously measured timestamp
        self.past = None


    def ode(self, x, t, u):
        """Dynamic equations of mass-spring-damper
        #Arguments:
            x: States of ode
            t: Timestamps
            u: Control signal value
        #Returns
            Derivative of states
        """
        # ODE of pendulum
        pos, vel = x
        dxdt = [vel, -(self.k/self.m)*pos - (self.b/self.m)*vel + (1.0/self.m)*u]
        return dxdt


    def update(self, u):
        """Interface function for mass-spring-damper model
        #Arguments:
            u: Control signal value
        #Returns:
            Position of mass
        """
        #Calculates output signal
        now = time()
        dt = 0.0
        if self.past is not None:
            dt = now - self.past
        self.past = now
        x = odeint(self.ode, self.x0, [0.0, dt], args=(u,))
        self.x0 = x[1,:]
        return x[1,0]