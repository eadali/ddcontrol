# -*- coding: utf-8 -*-
"""
Created on Wed Jan 15 10:06:42 2020

@author: ERADALI
"""
from numpy import array, zeros, ndarray, expand_dims, stack, ones
from numbers import Number
from scipy.signal import tf2ss
from scipy.optimize import curve_fit
from ddcontrol.integrate import CInterp1d, DDE



class StateSpace:
    """Time delayed linear system in state-space form.
    
    Args:
        A, B, C, D (array_like): State space matrices
        delays (array_like, optional): Delay values
    """    
    def __init__(self, A, B, C, D, delays=None):
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
            self.delays = array(delays, 'float32', copy=True)
        #Configures DDE
        self.solver = DDE(self.__ss_eq)
        self.set_initial_value()


    def set_initial_value(self, x0=None, u0=None):
        """Sets initial conditions 
        
        Args:        
            x0 (array_like or callable, optional): Initial state values
            u0 (array_like or callable, optional): Initial input values
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
        self.u = CInterp1d(u0, 0.0)
        #Creates states function
        self.x = CInterp1d(x0, 0.0)
        w = lambda t: array(x0(t), 'float32')
        self.solver.set_initial_value(0.0, w)


    ##TODO: find a clean solution for xd and ud
    def __ss_eq(self, t, x, u):
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


    ##TODO: find a clean solution for xd and ud
    def step(self, t, u):
        """Find dxdy = Ax + Bu, set x as an initial condition, 
        and return y = Cx + Du.
        
        Args:
            t (float): The endpoint of the integration step.
            u (float): Input value
        
        Returns
            array_like: Output of state space model
        """
        if t >= 0.0:
            #Appends new input value to input function
            self.u.append(t, u)
            #Sets input parameter
            self.solver.set_f_params((self.u,))
            #Solves dde
            self.x.append(t, self.solver.integrate(t))
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
    """Time delayed linear system in transfer function form.

    Args:
        num, den (array_like): Numerator and denumerator of the TransferFunction system.
        udelay (float, optional): Input delay value
    """
    def __init__(self, num, den, udelay=None):
        self.num, self.den = num, den
        self.udelay = udelay
        #Creates state space matrices
        A, B, C, D = tf2ss(num, den)
        if udelay is not None:
            A = stack((A,zeros(A.shape,'float32')))
            B = stack((zeros(B.shape,'float32'), B))
            C = stack((C, zeros(C.shape,'float32')))
            D = stack((D, zeros(D.shape,'float32')))
            udelay = [0.0, udelay]
        #Calls state space init function
        StateSpace.__init__(self, A, B, C, D, delays=udelay)


    def set_initial_value(self, u0=None):
        """Sets initial conditions.
        
        Args:        
            u0 (array_like or callable, optional): Initial input values.
        """
        #Creates input function
        if isinstance(u0, Number):
            uc = u0
            u0 = lambda t: array(uc, 'float32', copy=True)
        #Calls state space init functio
        StateSpace.set_initial_value(self, u0=u0)


    ##TODO: implement this function
    def to_ss(self):
        pass



def tfest(t, y, u, np, nz=None, delay=False, xtol=1e-4, epsfcn=1e-4):
    """Estimates a continuous-time transfer function.
    
    Args:
        t (float): The independent time variable where the data is measured.
        y (float): The dependent output data.
        u (float): The dependent input data.
        np (float): Number of poles.
        nz (float, optional): Number of zeros.
        delay (bool, optional): Status of input delay.
        xtol (float, optional): Relative error desired in the approximate solution.
        epsfcn (float, optional): A variable used in determining a suitable step length for
        the forward- difference approximation of the Jacobian

    Returns:
        tuple: Estimated TransferFunction and covariance ndarray
        
    Example:
        >>> from ddcontrol.model import TransferFunction, tfest
        >>> import numpy as np

        >>> #Creates a transfer function and input output data
        >>> tf = TransferFunction([1.0], [1.0,10.0,20.0])
        >>> t, y, u = np.linspace(0,10,101), np.zeros(101), np.ones(101)
        >>> for index in range(t.size):
        >>>     y[index] = tf.step(t[index], u[index])
        
        >>> #Predicts transfer function
        >>> tf, _ = tfest(t, y, u, np=2, nz=0)
        >>> print('Transfer function numerator coeffs..:', tf.num)
        >>> print('Transfer function denumerator coeffs..:', tf.den)
    
    Todo:
        Optimization is very slow. Improve performance.
        remove tf from mdl function
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
    _y = zeros((t.shape[0]), 'float32')
    def mdl(_u, *p):
        num, den, udelay = _reshape(p, nz, np, delay)
        tf = TransferFunction(num, den, udelay)
        for index in range(t.shape[0]):
            _y[index] = tf.step(t[index], u[index])
        return _y
    #Optimizes the transfer function gains
    popt, pcov = curve_fit(mdl, u, y, p0, xtol=xtol, epsfcn=epsfcn)
    #Creates num, den and delay values
    num, den, udelay = _reshape(popt, nz, np, delay)
    return TransferFunction(num, den, udelay), pcov