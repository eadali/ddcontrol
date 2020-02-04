# -*- coding: utf-8 -*-
"""
Created on Wed Jan 15 10:06:42 2020

@author: ERADALI
"""
from scipy.integrate import ode
from numpy import array, zeros, ndarray, expand_dims, stack, ones
from numbers import Number
from scipy.interpolate import interp1d
from scipy.signal import tf2ss
from collections import deque
from time import time
from scipy.integrate import odeint
from scipy.optimize import curve_fit



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
    def __init__(self, A, B, C, D, delays=None):
        """Inits state space matrices and solver
        #Arguments:
            A, B, C, D: State space matrices
            delays: Delay values
        """
        #Converts list to narray and fix to 3 dimension
        self.A = array(A, 'float32', copy=True)
        if self.A.ndim == 2:
            self.A = expand_dims(self.A, 0)
        self.B = array(B, 'float32', copy=True)
        if self.B.ndim == 2:
            self.B = expand_dims(self.B, 0)
        self.C = array(C, 'float32', copy=True)
        if self.C.ndim == 2:
            self.C = expand_dims(self.C, 0)
        self.D = array(D, 'float32', copy=True)
        if self.D.ndim == 2:
            self.D = expand_dims(self.D, 0)
        #Sets delays
        if delays is None:
            self.delays = array([0.0], 'float32')
        elif isinstance(delays, (list,tuple)):
            self.delays = array(delays, 'float32').copy()
        #Configures DDE
        self.solver = DDE(self._ss_eq)
        self.solver.set_integrator('dopri5')
        self.set_initial_value()


    def set_initial_value(self, x0=None, u0=None):
        """Inits states and inputs
        #Arguments
            x0: Initial states
            u0: Initial inputs
        """
        #Creates initial state function
        if x0 is None:
            x0 = lambda t: zeros(self.A.shape[2], 'float32')
        elif isinstance(x0, (list,tuple,ndarray)):
            xc = x0.copy()
            x0 = lambda t: array(xc, 'float32', copy=True)
        #Creates initial input function
        if u0 is None:
            u0 = lambda t: zeros(self.B.shape[2], 'float32')
        elif isinstance(u0, (list,tuple,ndarray)):
            uc = u0.copy()
            u0 = lambda t: array(uc, 'float32', copy=True)
        #Creates input function
        self.u = Interpolate(u0, 0.0)
        #Creates states function
        self.x = Interpolate(x0, 0.0)
        self.solver.set_initial_value(x0, 0.0)


    def _ss_eq(self, t, x, u):
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


    def integrate(self, t, u):
        """Integrates state space model
        #Arguments:
            t: Timestamp
            u: Input value
        #Returns
            Output of state space model
        """
        if t >= 0.0:
            #Appends new input value to input function
            self.u.append(t, u)
            #Sets input parameter
            self.solver.set_f_params((self.u,))
            #Solves dde
            self.x.append(t, self.solver.integrate(t))
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



class TransferFunction(StateSpace):
    def __init__(self, num, den, udelay=None):
        """Inits transfer function coeffs and solver
        #Arguments:
            num, den: Transfer function coeffs
            udelay: Input delay value
        """
        #Creates state space matrices
        A, B, C, D = tf2ss(num, den)
        if udelay is not None:
            A = stack((A,zeros(A.shape)))
            B = stack((zeros(B.shape), B))
            C = stack((C, zeros(C.shape)))
            D = stack((D, zeros(D.shape)))
            udelay = [0.0, udelay]
        #Calls state space init function
        StateSpace.__init__(self, A, B, C, D, delays=udelay)


    def set_initial_value(self, u0=None):
        """Inits states and inputs
        #Arguments
            u0: Initial inputs
        """
        #Creates input function
        if isinstance(u0, Number):
            uc = u0
            u0 = lambda t: array(uc, 'float32', copy=True)
        #Calls state space init functio
        StateSpace.set_initial_value(self, u0=u0)


    def to_ss(self):
        ##TODO: implement this function
        pass


def tfest(t, y, u, np, nz=None, delay=False):
    """Estimates transfer function coeffs
    #Arguments
        t: Timestamp values of measurments
        y: Measured output signal of system
        u: Applied control signal to system
        np: Number of poles
        nz: Number of zeros
        delay: Status of input delay
    #Returns
        Estimated numerator, denominator coeffs and covariance
    """
    #Timestamps start from zero
    t -= t.min()
    #If number of zeros is not given it is np-1
    if nz is None:
        nz = int(nz-1)
    #Creates initial prediction
    if delay:
        p0 = ones((nz+1)+(np+1)+1, 'float32')
    else:
        p0 = ones((nz+1)+(np+1), 'float32')
    #Reshape function for flattened array
    def _reshape(p, nz, np, delay):
        udelay = p[(nz+1)+(np+1)] if delay else None
        return p[0:(nz+1)], p[(nz+1):(nz+1)+(np+1)], udelay
    #Model for optimization
    def mdl(_, *p):
        num, den, udelay = _reshape(p, nz, np, delay)
        tf = TransferFunction(num, den, udelay)
        _y = zeros((t.shape[0]), 'float32')
        for index in range(t.shape[0]):
            _y[index] = tf.integrate(t[index], u[index])
        return _y
    #Optimizes the transfer function gains
    popt, pcov = curve_fit(mdl, u, y, p0, epsfcn=1e-5)
    #Creates num, den and delay values
    num, den, udelay = _reshape(popt, nz, np, delay)
    return (TransferFunction(num, den, udelay), pcov)


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


##TODO: Implement this function
#def estimate_ss(t, y, u, ns, nd=None):
#    nu = 0.0
#    ny = 0.0
#    p0 = ones(nd*ns*ns + nd*ns*nu + nd*ny*ns + nd*ny*nu + ns + nu 'float32')
#
#    def _reshape(p, nd, ns, nu, ny, nd):
#        A = p[:].reshape(nd, ns, ns)
#        B = p[:].reshape(nd, ns, nu)
#        C = p[:].reshape(nd, ny, ns)
#        D = p[:].reshape(nd, ny, nu)
#        return A, B, C, D, delays
#
#    def mdl(_, *p):
#        A, B, C, D, delays_reshape
#        ss = StateSpace(A, B, C, D, delays)
#        _y = zeros((t.shape[0],ny), 'float32')
#        for index in range(t.shape[0]):
#            _y[index] = ss.integrate(t[index], u[index])
#        return _y
#    popt, pcov = curve_fit(model, u_cont, y_meas, p0=init)
#    A = popt[:].reshape(nd, ns, ns)
#    B = popt[:].reshape(nd, ns, nu)
#    C = popt[:].reshape(nd, ny, ns)
#    D = popt[:].reshape(nd, ny, nu)
#    delays
#    return StateSpace(A, B, C, D, delays=delays)