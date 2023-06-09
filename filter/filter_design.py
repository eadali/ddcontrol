import numpy as np 

def std(t, x):
    noise = np.diff(x)
    return np.std(noise)

