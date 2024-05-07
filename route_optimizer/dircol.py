import numpy as np
import jax.numpy as jnp
from jax import jacfwd
from scipy.optimize import minimize

def calculate_trajectory(x0, xg, u_tw, v_tw, tf, dt=0.1):
    '''
    inputs:
        x0      initial state
        xg      goal state
        u_tw    true wind speed (latitudinal)
        v_tw    true wind speed (longitudinal)
        tf      final time
        dt      time step
    returns:
        X       trajectory
        U       control inputs
    '''