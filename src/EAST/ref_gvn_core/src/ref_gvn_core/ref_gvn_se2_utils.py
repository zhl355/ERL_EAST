#!/usr/bin/env python3
"""
Safe path-following controller based on reference governor

Interfaces:
    Input:
      nonlinear robot states (z), SE(2)
      strictly feasible path (r), 2d
      dist from governor to observed obstacle space (d_Q(g,O))
    Output:
      governor state (g) SE(2)
      
      # for visualization purpose
        local projected goal (lpg2d), 2d
        local safe zone (LS) ball/ellipse characteristics

"""


import numpy as np


class RefGvnError(Exception):
    """User Defined Exceptions for Reference Governor class."""

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "RefGvn exception: %s" % self.msg
        else:
            return "RefGvn exception"


class RK4Error(Exception):
    """ User Defined Exceptions.
    """

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "Runge Kutta 4th Order ODE Solver Error exception: {0}".format(self.msg)
        else:
            return "Runge Kutta 4th Order ODE Solver Error exception"


def feval(func_name, *args):
    # function value evaluation
    return np.array(func_name(*args))


def rk4_update(h, func, tn, yn, un, *extra_func_args):
    """
    RK4 takes one Runge-Kutta step.
    This function is used to solve the forced initial value ode of the form
    dy/dt = f(t, y, uvec),  with y(t0) = yn
    User need to supply current values of t, y, a step size h, external input
    vector, and  dynamics of ackerman drive to evaluate the derivative,
    this function can compute the fourth-order Runge Kutta estimate
    to the solution at time t+h.

    #  Parameters:
    INPUT:
        func,   function handle     RHS of ODE equation,
                                    specifically ackerman drive dynamics

        tn,     scalar              the current time.
        yn,     1D array            y value at time tn
        un,     1D array            external input at time tn
        h,      scalar              step size, scalar


    OUTPUT:
        yn1,    1D array            the 4_th order Runge-Kutta estimate
                                    y value at next time step
    """
    # Get four sample values of the derivative.
    k1 = feval(func, tn, yn, un, *extra_func_args)
    k2 = feval(func, tn + h / 2.0, yn + k1 * h / 2.0, un, *extra_func_args)
    k3 = feval(func, tn + h / 2.0, yn + k2 * h / 2.0, un, *extra_func_args)
    k4 = feval(func, tn + h, yn + k3 * h, un, *extra_func_args)

    # Estimate y at time t_n+1 using weight sum of 4 key sample pts
    yn1 = yn + h * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0

    return yn1


def integrator(t, y, u):
    """
    System dynamics for integrator.
    """
    dxdt = u

    return dxdt


def sat_range(lb, ub, val):
    """
    Restrict val within range [lb, ub]
    """
    if val > ub:
        val = ub
    if val < lb:
        val = lb
    return val
