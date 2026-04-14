# Helper functions for trajectory generation
import numpy as np

# Function for cosine interpolation
def cos_interp(t, z_start, z_end, t_start, t_end):
    '''Smooth cosine interpolation from z_start to z_end over [t_start, t_end]'''
    tau = (t - t_start) / (t_end - t_start)  # Normalized time 0->1
    return z_start + (z_end - z_start) * 0.5 * (1 - np.cos(np.pi * tau))

# Function for derivative of cosine interpolation
def cos_interp_dot(t, z_start, z_end, t_start, t_end):
    '''Derivative of cosine interpolation'''
    duration = t_end - t_start
    tau = (t - t_start) / duration
    return (z_end - z_start) * 0.5 * np.pi / duration * np.sin(np.pi * tau)